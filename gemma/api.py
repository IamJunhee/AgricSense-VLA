import os
import base64
import io
from fastapi import FastAPI, HTTPException, Body
from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from contextlib import asynccontextmanager
import logging
from PIL import Image

from pipeline import load_model, generate, generate_prompt

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

ADAPTER_MODEL_PATH = os.environ.get("ADAPTER_MODEL_PATH", "IamJunhee/Gemma3-Agricsense_lora")

class GenerateRequest(BaseModel):
    prompt: str = Field(..., description="사용자가 입력하는 텍스트 프롬프트")
    rgb_image_base64: Optional[str] = Field(None, description="Base64 인코딩된 RGB 이미지 데이터 (선택 사항)")
    d_image_base64: Optional[str] = Field(None, description="Base64 인코딩된 Depth 또는 다른 이미지 데이터 (선택 사항)")
    context: Optional[Dict[str, List[Dict[str, Any]]]] = Field(None, description="이전 대화 내용 (선택 사항)")
    max_new_tokens: int = Field(200, description="생성할 최대 새 토큰 수")

class GenerateResponseItem(BaseModel):
    generated: str = Field(..., description="모델이 생성한 텍스트")
    context: Dict[str, List[Dict[str, Any]]] = Field(..., description="업데이트된 대화 내용")

def pil_image_to_base64(image: Image.Image) -> str:
    """Converts a PIL Image object to a base64 encoded string with data URI."""
    buffered = io.BytesIO()
    img_format = image.format if image.format else "PNG"

    if image.mode == 'RGBA' or (image.mode == 'P' and 'transparency' in image.info):
        img_format = 'PNG'
    elif image.mode != 'RGB' and img_format != 'PNG':
        image = image.convert('RGB')

    try:
        image.save(buffered, format=img_format)
    except KeyError:
        img_format = "PNG"
        if image.mode != 'RGB' and image.mode != 'RGBA' and image.mode != 'L':
             image = image.convert('RGB')
        image.save(buffered, format=img_format)

    img_str = base64.b64encode(buffered.getvalue()).decode("utf-8")
    return f"data:image/{img_format.lower()};base64,{img_str}"

def decode_base64_image(base64_string: str) -> Optional[Image.Image]:
    if not base64_string:
        return None
    try:
        if "," in base64_string:
            base64_string = base64_string.split(',', 1)[1]
        image_bytes = base64.b64decode(base64_string)
        image = Image.open(io.BytesIO(image_bytes))
        return image
    except Exception as e:
        logger.error(f"Base64 이미지 디코딩 오류: {e}", exc_info=True)
        raise HTTPException(status_code=400, detail=f"잘못된 Base64 이미지 데이터입니다: {e}")

@asynccontextmanager
async def lifespan(app: FastAPI):
    logger.info(f"모델 로딩 시작: {ADAPTER_MODEL_PATH}")
    try:
        load_model(ADAPTER_MODEL_PATH)
        logger.info("모델 로딩 완료.")
    except Exception as e:
        logger.error(f"모델 로딩 중 오류 발생: {e}", exc_info=True)
        raise RuntimeError(f"필수 모델 로딩 실패: {e}") from e
    yield
    logger.info("애플리케이션 종료.")

app = FastAPI(lifespan=lifespan)

@app.get("/")
async def root():
    return {"message": "Gemma-3 4B Quantized Model API"}

@app.post("/generate", response_model=List[GenerateResponseItem])
async def handle_generate(request: GenerateRequest = Body(...)):
    logger.info(f"요청 수신: prompt='{request.prompt[:50]}...', context_exists={request.context is not None}, "
                f"rgb_image_provided={request.rgb_image_base64 is not None}, "
                f"d_image_provided={request.d_image_base64 is not None}")

    try:
        rgb_image_pil = decode_base64_image(request.rgb_image_base64)
        d_image_pil = decode_base64_image(request.d_image_base64)

        # generate_prompt는 PIL 이미지를 포함한 입력을 생성
        model_input = generate_prompt(
            prompt=request.prompt,
            rgb_image=rgb_image_pil,
            d_image=d_image_pil,
            context=request.context
        )

        # generate 함수는 PIL 이미지가 포함된 context를 가진 결과를 반환
        raw_results: List[Dict[str, Any]] = generate(
            prompt=model_input,
            max_new_tokens=request.max_new_tokens
        )

        # --- 결과 후처리: Context 내 PIL 이미지를 Base64로 변환 ---
        cleaned_results = []
        for item_dict in raw_results:
            context_to_clean = item_dict.get("context")
            if context_to_clean and isinstance(context_to_clean.get("messages"), list):
                for message in context_to_clean["messages"]:
                    if isinstance(message.get("content"), list):
                        new_content = []
                        for content_item in message["content"]:
                            if isinstance(content_item, dict) and content_item.get("type") == "image":
                                image_data = content_item.get("image")
                                if isinstance(image_data, Image.Image):
                                    try:
                                        # PIL 이미지를 Base64로 변환
                                        base64_str = pil_image_to_base64(image_data)
                                        # 새 아이템 딕셔너리 생성하여 Base64 문자열 저장
                                        cleaned_item = content_item.copy()
                                        cleaned_item["image"] = base64_str
                                        new_content.append(cleaned_item)
                                    except Exception as e:
                                        logger.error(f"Context 이미지 Base64 변환 오류: {e}", exc_info=True)
                                        new_content.append(content_item) # 실패 시 원본 유지 (혹은 다른 처리)
                                else:
                                    # PIL 이미지가 아니면 (이미 Base64 등) 그대로 추가
                                    new_content.append(content_item)
                            else:
                                # 이미지 타입이 아니면 그대로 추가
                                new_content.append(content_item)
                        # 메시지의 content를 업데이트
                        message["content"] = new_content

            # 처리된 (PIL 이미지가 Base64로 변환된) 아이템을 최종 결과 리스트에 추가
            cleaned_results.append(item_dict)
        # --- 결과 후처리 완료 ---

        logger.info(f"생성 및 Context 정제 완료: {len(cleaned_results)}개 결과")
        # 정제된 결과를 반환 (FastAPI가 이제 JSON으로 직렬화 가능)
        return cleaned_results

    except HTTPException as e:
        # FastAPI에서 발생시킨 HTTP 예외는 그대로 다시 발생시킴
        raise e
    except Exception as e:
        # 일반적인 예외 처리
        logger.error(f"생성 중 오류 발생: {e}", exc_info=True)
        if "Model is not loaded" in str(e):
             raise HTTPException(status_code=503, detail="모델이 아직 준비되지 않았습니다. 잠시 후 다시 시도해주세요.")
        # 그 외 내부 서버 오류
        raise HTTPException(status_code=500, detail=f"내부 서버 오류: {str(e)}")
