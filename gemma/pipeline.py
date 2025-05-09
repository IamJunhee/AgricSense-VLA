from PIL import Image
import numpy as np
import torch
from copy import deepcopy
from transformers import AutoProcessor, AutoModelForImageTextToText, BitsAndBytesConfig
from peft import PeftModel
from typing import Union, Optional, List, Dict, Any # 추가

__model = None
__processor = None

def load_model(model_path: str):
    global __model
    global __processor

    model_id = "google/gemma-3-4b-it"

    if __model is not None:
        return

    model_kwargs = dict(
        torch_dtype=torch.float32,
        device_map="auto",
    )

    if torch.cuda.is_available() and torch.cuda.get_device_capability()[0] >= 8:
        model_kwargs["torch_dtype"] = torch.bfloat16
        model_kwargs["attn_implementation"] = "flash_attention_2"

    model_kwargs["quantization_config"] = BitsAndBytesConfig(
        load_in_4bit=True,
        bnb_4bit_use_double_quant=True,
        bnb_4bit_quant_type="nf4",
        bnb_4bit_compute_dtype=model_kwargs["torch_dtype"],
        bnb_4bit_quant_storage=model_kwargs["torch_dtype"],
    )

    __model = AutoModelForImageTextToText.from_pretrained(model_id, **model_kwargs)
    __model = PeftModel.from_pretrained(__model, model_path)
    __processor = AutoProcessor.from_pretrained(model_id, use_fast=True)

def load_and_process_image(image: Union[str, Image.Image]) -> Image.Image:
    if isinstance(image, str):
        try:
            image = Image.open(image)
        except FileNotFoundError:
            raise FileNotFoundError(f"지정된 경로에 파일이 없습니다: {image}")
    elif not isinstance(image, Image.Image):
        raise TypeError("이미지는 파일 경로(str) 또는 PIL.Image 객체여야 합니다.")

    channels = len(image.getbands())

    if channels == 1:
        img = np.array(image)
        height, width = img.shape
        three_channel_array = np.zeros((height, width, 3), dtype=np.uint8)

        if img.dtype == np.uint8:
            img = img.astype(np.uint16)
            img = ((img / 255) * 65535).astype(np.uint16)

        three_channel_array[:, :, 0] = (img // 1024) * 2
        three_channel_array[:, :, 1] = (img // 32) * 8
        three_channel_array[:, :, 2] = (img % 32) * 8
        image = Image.fromarray(three_channel_array, "RGB")
    elif image.mode != "RGB":
        image = image.convert("RGB")

    return image

def process_vision_info(messages: List[Dict[str, Any]]) -> List[Image.Image]:
    image_inputs_to_process = []
    for msg in messages:
        content = msg.get("content", [])
        if not isinstance(content, list):
            content = [content]

        for element in content:
            if isinstance(element, dict) and element.get("type") == "image":
                image_input = None
                if "path" in element:
                    image_input = element["path"]
                elif "image" in element and isinstance(element.get("image"), Image.Image): # .get 추가
                    image_input = element["image"]

                if image_input is not None:
                    image_inputs_to_process.append(image_input)
                else:
                    # print(f"경고: type이 'image'이지만 'path' 또는 'image' 키/값을 찾을 수 없습니다: {element}")
                    pass # 로깅 등으로 대체 가능

    return [load_and_process_image(input_data) for input_data in image_inputs_to_process]

def collate_data(datas: Union[Dict[str, Any], List[Dict[str, Any]]], processor: AutoProcessor, for_generation: bool = True) -> Dict[str, torch.Tensor]:
    texts = []
    images = []

    if not isinstance(datas, list):
        datas = [datas]

    for data in datas:
        image_objects = process_vision_info(data["messages"])
        text = processor.apply_chat_template(
            data["messages"], add_generation_prompt=for_generation, tokenize=False
        )
        texts.append(text.strip())
        images.append(image_objects)

    images_arg = images if any(img_list for img_list in images) else None
    batch = processor(text=texts, images=images_arg, return_tensors="pt", padding=True)

    return batch

def collate_data_for_train(datas: Union[Dict[str, Any], List[Dict[str, Any]]], processor: AutoProcessor) -> Dict[str, torch.Tensor]:
    batch = collate_data(datas, processor, False)
    labels = batch["input_ids"].clone()

    # special_tokens_map에서 'boi_token'이 없을 경우 대비
    image_token_id_value = processor.tokenizer.convert_tokens_to_ids(
        processor.tokenizer.special_tokens_map.get("boi_token", "<image>") # 기본값으로 <image> 사용 또는 에러 처리
    )
    image_token_id = [image_token_id_value]


    labels[labels == processor.tokenizer.pad_token_id] = -100
    labels[labels == image_token_id_value] = -100 # 변수 사용
    # 특정 토큰 ID (262144) 처리 - Gemma 3 모델의 특수 토큰일 수 있음
    labels[labels == 262144] = -100

    batch["labels"] = labels
    return batch

def generate_prompt(prompt: str, rgb_image: Optional[Image.Image] = None, d_image: Optional[Image.Image] = None, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]: # context 타입 힌트 수정
    if context is None:
        result = dict(messages = [ ])
    else:
        result = deepcopy(context)

    content: List[Dict[str, Any]] = [] # 타입 힌트 추가
    if rgb_image is not None:
        content.append(
            {
                "type": "image",
                "image": rgb_image
            }
        )
    if d_image is not None:
        content.append(
            {
                "type": "image",
                "image": d_image
            }
        )
    content.append(
        {
            "type": "text",
            "text": prompt
        }
    )

    result["messages"].append(
        {
            "role" : "user",
            "content" : content
        }
    )

    return result

generate_prompt_with_context = lambda prompt, context: generate_prompt(prompt, None, None, context)

def generate(prompt: Union[Dict[str, Any], List[Dict[str, Any]]], max_new_tokens: int = 200) -> List[Dict[str, Any]]: # 반환 타입 힌트 수정
    global __model
    global __processor

    if __model is None or __processor is None:
        raise Exception("Model is not loaded")

    batch = collate_data(prompt, __processor, for_generation=True).to(__model.device, dtype=__model.config.torch_dtype if hasattr(__model.config, 'torch_dtype') else torch.float32) # dtype 접근 안전성 추가

    input_len = batch["input_ids"].shape[-1]

    with torch.inference_mode():
        generation_output = __model.generate(**batch, max_new_tokens=max_new_tokens, top_k=64, top_p=0.95, do_sample=True)
        generated_tokens = [result[input_len:] for result in generation_output]
    decoded = [__processor.decode(result, skip_special_tokens=True).strip("\n") for result in generated_tokens]

    context_list = deepcopy(prompt)

    if not isinstance(context_list, list):
        context_list = [context_list]

    output_list = [] # 결과를 담을 리스트 초기화
    for index in range(len(context_list)):
        current_context = context_list[index]
        current_decoded = decoded[index]

        current_context["messages"].append(
            {
                "role" : "assistant",
                "content" : [
                    {
                        "type": "text",
                        "text": current_decoded
                    }
                ]
            }
        )
        output_list.append(dict(generated=current_decoded, context=current_context)) # 결과 리스트에 추가

    return output_list # 최종 결과 리스트 반환