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

        model_input = generate_prompt(
            prompt=request.prompt,
            rgb_image=rgb_image_pil,
            d_image=d_image_pil,
            context=request.context
        )

        results = generate(
            prompt=model_input,
            max_new_tokens=request.max_new_tokens
        )

        logger.info(f"생성 완료: {len(results)}개 결과")
        return results

    except HTTPException as e:
        raise e
    except Exception as e:
        logger.error(f"생성 중 오류 발생: {e}", exc_info=True)
        if "Model is not loaded" in str(e):
             raise HTTPException(status_code=503, detail="모델이 아직 준비되지 않았습니다. 잠시 후 다시 시도해주세요.")
        raise HTTPException(status_code=500, detail=f"내부 서버 오류: {e}")