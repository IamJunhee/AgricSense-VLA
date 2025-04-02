#!/usr/bin/env python
# coding: utf-8

# In[1]:


import os
from huggingface_hub import login

hf_token = os.environ['HF_TOKEN']
login(hf_token)


# In[2]:


from PIL import Image
from io import BytesIO
import numpy as np
import base64

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
    # im_file = BytesIO()
    # image.save(im_file, format="JPEG")
    # im_bytes = im_file.getvalue()
    
    # return base64.b64encode(im_bytes)


# In[3]:


def conversation_to_template(data: dict) -> dict:
    image_path = lambda index : os.path.join(get_dataset_dir(SPLIT_NUMBER), data["image"][index])
    
    result = { "messages" : [ ] }
    result["messages"].append(
        {
            "role" : "user",
            "content" : [
                {
                    "type": "image",
                    "path": image_path(0)
                },
                {
                    "type": "image",
                    "path": image_path(1)
                },
                {
                    "type": "text",
                    "text": data["conversations"][0]["value"].split("\n")[2]
                }
            ]
        }
    )

    for ele in data["conversations"][1:]:
        msg = {
            "role": None,
            "content" : []
        }

        msg["role"] = "assistant" if ele["from"] == "gpt" else "user"
        msg["content"].append(
            {
                "type" : "text",
                "text" : ele["value"]
            }
        )
        result["messages"].append(msg)

    return result


# In[4]:


from datasets import load_dataset

DATASET_BASE = "/home/junhee0110/dataset/images_split_{}" #"/home/iamjunhee/OhMyData/SpatialQA/dataset/images_split_{}"
JSON_BASE = "NewQA_split_{}.json"
SPLIT_NUMBER = 1


get_dataset_dir = lambda index: DATASET_BASE.format(index)
get_json_dir = lambda index: os.path.join(DATASET_BASE.format(index), JSON_BASE.format(index))

dataset = load_dataset("json", data_files=get_json_dir(SPLIT_NUMBER), split="train")
dataset = [conversation_to_template(sample) for sample in dataset]


# In[5]:


import torch
from transformers import AutoProcessor, AutoModelForImageTextToText, BitsAndBytesConfig

# Hugging Face model id
model_id = "google/gemma-3-4b-it"

# Check if GPU benefits from bfloat16
if torch.cuda.get_device_capability()[0] < 8:
    raise ValueError("GPU does not support bfloat16, please use a GPU that supports bfloat16.")

# Define model init arguments
model_kwargs = dict(
    attn_implementation="flash_attention_2", # Use "flash_attention_2" when running on Ampere or newer GPU
    torch_dtype=torch.bfloat16, # What torch dtype to use, defaults to auto
    device_map="auto", # Let torch decide how to load the model
)

# BitsAndBytesConfig int-4 config
model_kwargs["quantization_config"] = BitsAndBytesConfig(
    load_in_4bit=True,
    bnb_4bit_use_double_quant=True,
    bnb_4bit_quant_type="nf4",
    bnb_4bit_compute_dtype=model_kwargs["torch_dtype"],
    bnb_4bit_quant_storage=model_kwargs["torch_dtype"],
)

# Load model and tokenizer
model = AutoModelForImageTextToText.from_pretrained(model_id, **model_kwargs)
processor = AutoProcessor.from_pretrained("google/gemma-3-4b-it", use_fast=True)


# In[6]:


model


# In[7]:


from peft import LoraConfig

peft_config = LoraConfig(
    lora_alpha=32,
    lora_dropout=0.05,
    r=16,
    bias="none",
    target_modules=[
        "q_proj",
        "k_proj",
        "v_proj",
        "o_proj",
        "gate_proj",
        "up_proj",
        "down_proj",
        "fc1",
        "fc2",
        "out_proj",
        "lm_head",
    ],
    task_type="CAUSAL_LM",
    modules_to_save=[
    ],
)


# In[8]:


from trl import SFTConfig

args = SFTConfig(
    output_dir="gemma-product-description",     # directory to save and repository id
    num_train_epochs=3,                         # number of training epochs
    per_device_train_batch_size=2,              # batch size per device during training
    gradient_accumulation_steps=4,              # number of steps before performing a backward/update pass
    gradient_checkpointing=True,                # use gradient checkpointing to save memory
    optim="adamw_torch_fused",                  # use fused adamw optimizer
    logging_steps=5,                            # log every 5 steps
    save_strategy="epoch",                      # save checkpoint every epoch
    learning_rate=2e-4,                         # learning rate, based on QLoRA paper
    bf16=True,                                  # use bfloat16 precision
    max_grad_norm=0.3,                          # max gradient norm based on QLoRA paper
    warmup_ratio=0.03,                          # warmup ratio based on QLoRA paper
    lr_scheduler_type="constant",               # use constant learning rate scheduler
    push_to_hub=False,                           # push model to hub
    report_to="tensorboard",                    # report metrics to tensorboard
    gradient_checkpointing_kwargs={
        "use_reentrant": False
    },  # use reentrant checkpointing
    dataset_text_field="",                      # need a dummy field for collator
    dataset_kwargs={"skip_prepare_dataset": True},  # important for collator
)
args.remove_unused_columns = False # important for collator


# In[9]:


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


# In[10]:


# Create a data collator to encode text and image pairs
def collate_fn(examples):
    texts = []
    images = []
    for example in examples:
        image_inputs = process_vision_info(example["messages"])
        text = processor.apply_chat_template(
            example["messages"], add_generation_prompt=False, tokenize=False
        )
        texts.append(text.strip())
        images.append(image_inputs)

    # Tokenize the texts and process the images
    batch = processor(text=texts, images=images, return_tensors="pt", padding=True)

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


# In[11]:


from trl import SFTTrainer

trainer = SFTTrainer(
    model=model,
    args=args,
    train_dataset=dataset,
    peft_config=peft_config,
    processing_class=processor,
    data_collator=collate_fn,
)


# In[ ]:


# Start training, the model will be automatically saved to the Hub and the output directory
trainer.train()

# Save the final model again to the Hugging Face Hub
trainer.save_model()


# In[ ]:




