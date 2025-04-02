from PIL import Image
import numpy as np
import torch
from copy import deepcopy

def load_and_process_image(path: str) -> str:
    image = Image.open(path)
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

    return image

def process_vision_info(messages: list[dict]) -> list[Image.Image]:
    image_inputs = []
    # Iterate through each conversation
    for msg in messages:
        # Get content (ensure it's a list)
        content = msg.get("content", [])
        if not isinstance(content, list):
            content = [content]

        # Check each content element for images
        for element in content:
            if isinstance(element, dict) and (
                "image" in element or element.get("type") == "image"
            ):
                # Get the image and convert to RGB
                if "path" in element:
                    image = element["path"]
                else:
                    image = element
                image_inputs.append(image)

    return [load_and_process_image(input).convert("RGB") for input in image_inputs]

def collate_data(datas, processor, for_generation=False):
    texts = []
    images = []

    if not isinstance(datas, list):
        datas = [datas]

    for data in datas:
        image_inputs = process_vision_info(data["messages"])
        text = processor.apply_chat_template(
            data["messages"], add_generation_prompt=for_generation, tokenize=False
        )
        texts.append(text.strip())
        images.append(image_inputs)

    # Tokenize the texts and process the images
    batch = processor(text=texts, images=images, return_tensors="pt", padding=True)

    return batch

def collate_data_for_train(datas, processor):
    batch = collate_data(datas, processor)

    # The labels are the input_ids, and we mask the padding tokens and image tokens in the loss computation
    labels = batch["input_ids"].clone()

    # Mask image tokens
    image_token_id = [
        processor.tokenizer.convert_tokens_to_ids(
            processor.tokenizer.special_tokens_map["boi_token"]
        )
    ]
    # Mask tokens for not being used in the loss computation
    labels[labels == processor.tokenizer.pad_token_id] = -100
    labels[labels == image_token_id] = -100
    labels[labels == 262144] = -100

    batch["labels"] = labels
    return batch

def generate_prompt(prompt: str, rgb_path: str, d_path: str, context: dict = None) -> dict:
    if context is None:
        result = dict(messages = [ ])
    else:
        result = deepcopy(context)

    content = []
    if rgb_path is not None:
        content.append(
            {
                "type": "image",
                "path": rgb_path
            }
        )
    if d_path is not None:
        content.append(
            {
                "type": "image",
                "path": d_path
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

def generate(prompt, model, processor, max_new_tokens=200) -> dict:
    batch = collate_data(prompt, processor, for_generation=True).to(model.device, dtype=model.config.torch_dtype)
    
    input_len = batch["input_ids"].shape[-1]
        
    with torch.inference_mode():
        generation = model.generate(**batch, max_new_tokens=max_new_tokens)
        generation = [result[input_len:] for result in generation]
    decoded = [processor.decode(result, skip_special_tokens=True).strip("\n") for result in generation]

    context = deepcopy(prompt)
    
    for index in range(len(context)):
        context[index]["messages"].append(
            {
                "role" : "assistant",
                "content" : [
                    {
                        "type": "text",
                        "text": decoded[index]
                    }
                ]
            }
        )

    return dict(generated=decoded, context=context)
    