import asyncio
import websockets
import json
import base64
import cv2 
import numpy as np
import os
import traceback
from datetime import datetime, timezone
from typing import Optional, Dict, Any, List, Callable

from google import genai
from google.genai import types

WEBSOCKET_URI = "ws://localhost:9090"
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
GEMINI_MODEL_NAME = "gemini-2.5-flash-preview-04-17"
THINKING_BUDGET = 4048

SYSTEM_PROMPT_COGNITION = """귀하의 임무는 농장 로봇 VLA 시스템의 '인지(Perception)' 단계를 위한 고품질 학습 데이터를 생성하는 것입니다.
주어진 프롬프트 내 이미지 설명(만약 있다면), 로봇의 현재 위치, 주변 농장 정보(`farm_info`), 이전 행동, 그리고 현재 사용자 명령을 종합적으로 고려하십시오.
이를 바탕으로 로봇의 현재 환경, 관찰되는 주요 객체, 명확하거나 잠재적인 장애물, 그리고 전반적인 상황 및 현재 작업 진행 상황을 포괄적이고 정확하며 상세한 텍스트로 기술해주십시오.
생성된 설명은 다음 '사고(Reasoning)' 단계에서 효과적인 계획을 수립할 수 있도록 완전하고 명확한 컨텍스트를 제공해야 하며, AI 모델(Gemma 4B)이 주변 환경을 깊이 이해하는 방법을 학습하는 데 사용될 최상의 예시가 되어야 합니다.
'OUTPUT (자세하게):' 지시에 따라 응답해주십시오.
"""

SYSTEM_PROMPT_REASONING = """귀하의 임무는 농장 로봇 VLA 시스템의 '사고(Reasoning/Planning)' 단계를 위한 고품질 학습 데이터를 생성하는 것입니다.
앞선 '인지' 단계에서 제공된 현재 상황 분석 결과와 전체 사용자 명령을 기반으로, 로봇이 지능적이고 효율적으로 작업을 수행하는 모범적인 다단계 추론 과정을 보여주십시오.
생성 시 다음의 '결정 정책'들을 적극적으로 반영하여 계획을 수립해야 합니다:

1.  최종 목표 달성을 위해 전체 작업 흐름을 고려하고, 여러 단계 앞을 내다보는 장기 계획을 논리적인 중간 목표들과 함께 수립합니다.
2.  `farm_info`를 활용해 최적 경로를 선택하고, 'move'(장거리/특정 좌표), 'spin'(방향 전환/관찰/정렬), 'forward'(단거리 직선/정렬 후 이동) 함수를 상황에 맞게 효율적으로 조합하여 사용합니다. 불필요한 이동이나 복잡한 행동 순서는 지양합니다.
3.  farm_info에는 농장의 모든 정보를 담고 있지는 않습니다. 알고있는 장소라면 move를 사용하는 것이 좋고, 모르는 장소라면 spin과 forward를 활용하는 것이 좋습니다.
4.  move의 매개변수들을 farm_info를 그대로 활용하는 것도 좋지만, 좌표와 각도 값을 한번 더 생각해서 조정하는 것이 효율적인 경우가 많습니다.
5.  알고 있는 장소의 좌표를 활용해서 move를 사용하는 것은 좋습니다. 하지만 angle은 현재 방향에 맞도록 수정해서 action을 도출하는 것이 좋습니다. 

위 정책들을 바탕으로, '인지' 단계 결과와 사용자 명령에 따라 다음 질문들에 대한 답변을 포함하여 상세한 계획을 작성해주십시오:
    1.  지금까지 어떤 작업을 수행했는가? (현재까지의 진행 상황과 주요 달성 내용 요약)
    2.  최종 목표(사용자 명령) 달성을 위해, 앞으로 어떤 주요 단계들(중간 목표들)을 순차적으로 거쳐야 하며, 각 주요 단계에서 달성해야 할 세부 목표는 무엇인가? (장기적인 계획 포함)
    3.  이러한 장기적 계획 하에, 지금 당장 수행해야 할 가장 구체적이고 적절한 다음 행동은 무엇인가? (그 이유는 무엇이며, 어떤 결정 정책(들)이 어떻게 고려되었는가?)

생성된 사고 과정은 AI 모델(Gemma 4B)이 복잡한 문제를 해결하고, 안전하며 효율적인 판단을 내리는 방법을 학습하는 데 사용될 최상의 예시가 되어야 합니다.
'OUTPUT (자세하게):' 지시에 따라 응답해주십시오.
"""

SYSTEM_PROMPT_ACTION = """귀하의 임무는 농장 로봇 VLA 시스템의 '행동(Action)' 단계를 위한 고품질 학습 데이터를 생성하는 것입니다.
앞선 '인지' 및 '사고' 단계에서 분석된 정보와 수립된 행동 계획을 바탕으로, 사용 가능한 함수 목록 중에서 가장 적절한 단일 행동(함수)을 선택하고, 정확한 파라미터를 포함하여 **지정된 JSON 객체 형식으로만 응답**해주십시오.
JSON 객체 외부에 어떠한 추가적인 설명, 인사, 일반 텍스트도 절대 포함해서는 안 됩니다. 오직 순수한 JSON 객체 하나만을 출력해야 합니다.

행동 선택 시 다음 '결정 정책'의 최종 적용을 고려해야 합니다:
1.  **움직임 효율성:** '사고' 단계에서 계획된 움직임('move', 'spin', 'forward'의 적절한 사용)을 정확한 함수 호출로 변환합니다.
2.  행동에 필요한 Parameter에 대해서 깊게 생각해보세요. 주어진 정보를 융합하여 새로운 지점으로 이동이 가능합니다.
3.  가령, 현재 위치를 그대로 move에 넣고, angle만을 바꾸면 spin과 비슷한 효과를 낼 수 있습니다. (spin이 잘 되지 않을 때 시도해보세요)
4.  farm_info의 주요 지점의 좌표로 이동을 하면서, angle값을 수정하면 다양한 장면을 얻어낼 수 있습니다.
5.  move를 활용할때, angle값은 farm_info의 내용을 그대로 사용해도 좋지만, 이동 방향과 angle를 나란하게 두는 것이 더 자연스러운 움직임을 만듭니다.
6.  15도 이하의 각도는 너무 작아서 의미가 없기도 하지만, 너무 작은 각도는 회전이 안될 가능성이 크다는 것을 기억하세요

JSON 객체 내 필드는 다음과 같이 정확히 채워주십시오:
-   `scene` 필드: 현재 상황에 대한 핵심적인 요약 (주로 '인지' 단계 결과 및 현재 행동 결정에 가장 직접적으로 관련된 정보 위주).
-   `action` 필드: 선택된 함수명(`name`)과 필요한 모든 파라미터(`parameters`).
-   `reason` 필드: 해당 행동(함수 및 파라미터)을 선택한 명확하고 구체적인 논리적 근거를 상세히 기술합니다. 이 근거는 '사고' 단계의 계획 및 고려된 결정 정책(예: 안전, 효율성, 특정 함수 선택의 타당성)과 일치해야 합니다.

생성된 JSON 출력은 AI 모델(Gemma 4B)이 추론 결과를 바탕으로 정확하고 근거 있는 행동을 생성하는 방법을 학습하는 데 사용될 최상의 예시가 되어야 합니다.
'Output (JSON Object Only):' 지시에 따라 응답해주십시오.
"""

gemini_client : Optional[genai.client.Client] = None
if genai and GEMINI_API_KEY:
    try:
        gemini_client = genai.Client(api_key=GEMINI_API_KEY)
        print("[Main] Gemini client initialized successfully.")
    except Exception as e:
        print(f"[Main] Error initializing Gemini client: {e}")
        gemini_client = None
elif not genai:
    print("[Main] Gemini SDK (google.genai) not imported. API calls cannot be made.")
else: 
    print("[Main] Warning: GEMINI_API_KEY not set. Real API calls will fail.")


def get_depth_value_from_base64_png_opencv(
    base64_depth_data: str,
    scale_from_meta: float,
    unit_from_meta: str,
    pixel_x: int, 
    pixel_y: int
) -> float:
    """Gets depth in meters at (x,y) pixel of the RGB image."""
    print(f"[DepthTool] Called with pixel_x={pixel_x}, pixel_y={pixel_y}, unit={unit_from_meta}, scale={scale_from_meta}")
    try:
        image_data_bytes = base64.b64decode(base64_depth_data)
        np_arr = np.frombuffer(image_data_bytes, np.uint8)
        cv_depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

        if cv_depth_image is None: 
            print("[DepthTool] Error: cv_depth_image is None after decoding.")
            return -1.0 
        if not (0 <= pixel_y < cv_depth_image.shape[0] and 0 <= pixel_x < cv_depth_image.shape[1]): 
            print(f"[DepthTool] Error: Pixel ({pixel_x},{pixel_y}) out of bounds ({cv_depth_image.shape[1]}x{cv_depth_image.shape[0]}).")
            return -2.0

        raw_depth_value = cv_depth_image[pixel_y, pixel_x]
        print(f"[DepthTool] Raw depth value at ({pixel_x},{pixel_y}): {raw_depth_value}")
        scaled_depth_value = float(raw_depth_value) * scale_from_meta

        if unit_from_meta.lower() == "mm": depth_meters = scaled_depth_value / 1000.0
        elif unit_from_meta.lower() == "m": depth_meters = scaled_depth_value
        else: depth_meters = scaled_depth_value 
        print(f"[DepthTool] Calculated depth: {depth_meters:.3f} meters.")
        return depth_meters
    except Exception as e:
        print(f"[DepthTool] Exception: {e}")
        return -3.0

async def call_gemini_api_live( 
    stage: str, 
    contents_for_api: List[types.Content | Dict[str, Any]],
    generation_config_override: Optional[types.GenerateContentConfig] = None # 여기서는 types.GenerateContentConfig 사용
):
    if not gemini_client:
        print(f"[GeminiAPI] Error: Gemini client not initialized for stage '{stage}'.")
        return {"error": "Gemini client not initialized.", "generated_text": f"[No Client for {stage}]", "full_response_content": None}

    # final_generation_config는 GenerateContentConfig 객체여야 합니다.
    final_generation_config = generation_config_override if generation_config_override else types.GenerateContentConfig()
    
    print(f"[GeminiAPI] Calling for stage '{stage}'. Contents length: {len(contents_for_api)}. Config: {final_generation_config}")
    try:
        response = await asyncio.to_thread(
            gemini_client.models.generate_content, # gemini_client.models 사용
            model=GEMINI_MODEL_NAME,
            contents=contents_for_api, # type: ignore
            config=final_generation_config, # config 대신 generation_config 사용
            # request_options={'timeout': 120} # 필요시 추가
        )
        
        generated_text = ""
        print(f"[GeminiAPI] Response received for stage '{stage}'.")

        if response.text: 
            print(f"[GeminiAPI] Stage '{stage}': Response.text available.")
            generated_text = response.text
        elif response.candidates and response.candidates[0].content and response.candidates[0].content.parts:
            print(f"[GeminiAPI] Stage '{stage}': Processing response.candidates.")
            for part_idx, part in enumerate(response.candidates[0].content.parts):
                if part.text:
                    generated_text += part.text
                # if hasattr(part, 'function_call') and part.function_call:
                    # print(f"[GeminiAPI] Stage '{stage}', Part {part_idx}: Detected function call: {part.function_call.name}")
        # else:
            # print(f"[GeminiAPI] Stage '{stage}': No text found in response.text or response.candidates.")
        
        print(f"[GeminiAPI] Stage '{stage}', Generated text (first 100 chars): {generated_text.strip()[:100] if generated_text else 'None'}")
        
        return {
            "generated_text": generated_text.strip() if generated_text else None, 
            "tool_calls": None, # 자동 함수 호출 사용 시 SDK가 처리, 명시적 tool_calls는 없을 것으로 가정
            "full_response_content": response.candidates[0].content if response.candidates else None
        }

    except Exception as e:
        print(f"[GeminiAPI] Error calling Gemini API for stage '{stage}': {e}")
        traceback.print_exc() # 상세 오류 출력
        return {"error": str(e), "generated_text": f"[Error during {stage}: {e}]", "tool_calls": None, "full_response_content": None}

async def gemini_data_synthesis_agent():
    print(f"[AgentLoop] Gemini Synthesizer: Attempting to connect to {WEBSOCKET_URI}...")
    async with websockets.connect(WEBSOCKET_URI, max_size=10 * (2 ** 20)) as websocket:
        print(f"[AgentLoop] Gemini Synthesizer: Successfully connected to {WEBSOCKET_URI}")
        
        current_depth_image_metadata: Optional[Dict[str, Any]] = None
        current_base64_depth_data: Optional[str] = None

        while True:
            print("[AgentLoop] Waiting for message from VLA server...")
            message_str = await websocket.recv()
            message = json.loads(message_str)
            print(f"[AgentLoop] Received message type: {message.get('type')}")

            if message.get("type") == "sceneData":
                payload = message.get("payload", {})
                prompts = payload.get("prompts", {})
                vision_data = payload.get("vision", {})
                
                rgb_image_info = vision_data.get("rgbImage", {})
                base64_rgb = rgb_image_info.get("data")
                rgb_format = rgb_image_info.get("format", "jpeg")

                current_base64_depth_data = vision_data.get("depthImage", {}).get("data")
                current_depth_image_metadata = vision_data.get("depthImage", {})
                
                timestamp = vision_data.get("imageTimestamp", datetime.now(timezone.utc).isoformat())
                print(f"[AgentLoop] Processing sceneData for timestamp: {timestamp}")
                gemini_responses_for_vla: Dict[str, str] = {}
                
                current_chat_history: List[types.Content | Dict[str, Any]] = [] 

                def dynamic_depth_tool(pixel_x: int, pixel_y: int) -> Dict[str, Any]: 
                    """Gets depth in meters at (x,y) pixel of the RGB image from the current scene. The (x,y) coordinates are pixel coordinates from the top-left of the image."""
                    print(f"[dynamic_depth_tool] Invoked for x={pixel_x}, y={pixel_y}")
                    if not current_base64_depth_data or not current_depth_image_metadata:
                        print("[dynamic_depth_tool] Error: Depth data not available in current scope.")
                        return {"error": "Depth data not available", "depth_meters": -99.0}
                    
                    depth_m = get_depth_value_from_base64_png_opencv(
                        current_base64_depth_data,
                        # current_depth_image_metadata.get("encoding", "16UC1"), # encoding 인자 제거됨
                        float(current_depth_image_metadata.get("scale", 1.0)),
                        current_depth_image_metadata.get("unit", "mm"),
                        pixel_x, pixel_y
                    )
                    print(f"[dynamic_depth_tool] Result for ({pixel_x},{pixel_y}): {depth_m} meters")
                    return {"depth_meters": depth_m}

                # --- Stage 1: Cognition ---
                print("[AgentLoop] Stage 1: Cognition")
                cognition_prompt = prompts.get("cognition", "")
                user_cognition_parts: List[types.Part] = []
                if base64_rgb:
                    try: user_cognition_parts.append(types.Part.from_bytes(data=base64.b64decode(base64_rgb), mime_type=f"image/{rgb_format}")) # from_bytes 사용
                    except Exception as e: print(f"[AgentLoop] Error making RGB Part: {e}")
                user_cognition_parts.append(types.Part.from_text(text = cognition_prompt))
                
                if user_cognition_parts: current_chat_history.append(types.UserContent(parts=user_cognition_parts)) # UserContent 사용
                
                cognition_gen_config = types.GenerateContentConfig(
                    tools=[dynamic_depth_tool],
                    thinking_config=types.ThinkingConfig(thinking_budget=THINKING_BUDGET),
                    system_instruction=SYSTEM_PROMPT_COGNITION
                )

                print("[AgentLoop] Calling Gemini for Cognition...")
                api_cognition_result = await call_gemini_api_live(
                    stage="cognition",
                    contents_for_api=current_chat_history,
                    generation_config_override=cognition_gen_config 
                )
                
                if api_cognition_result.get("full_response_content"):
                    current_chat_history.append(api_cognition_result["full_response_content"])
                
                gemini_responses_for_vla["cognition"] = api_cognition_result.get("generated_text", "Error.")
                if not gemini_responses_for_vla["cognition"] and api_cognition_result.get("error"):
                    gemini_responses_for_vla["cognition"] = api_cognition_result.get("error", "Unknown cognition error")
                print(f"[AgentLoop] Cognition result (first 50 chars): {gemini_responses_for_vla['cognition'][:50] if gemini_responses_for_vla['cognition'] else 'None'}")

                # --- Stage 2: Reasoning ---
                print("[AgentLoop] Stage 2: Reasoning")
                reasoning_prompt = prompts.get("reasoning", "")
                current_chat_history.append(types.UserContent(parts=[types.Part.from_text(text = reasoning_prompt)]))

                reasoning_generation_config = types.GenerateContentConfig(
                    thinking_config=types.ThinkingConfig(thinking_budget=THINKING_BUDGET),
                    system_instruction=SYSTEM_PROMPT_REASONING
                )
                
                print("[AgentLoop] Calling Gemini for Reasoning...")
                api_reasoning_result = await call_gemini_api_live(
                    stage="reasoning",
                    generation_config_override=reasoning_generation_config,
                    contents_for_api=current_chat_history
                )
                gemini_responses_for_vla["reasoning"] = api_reasoning_result.get("generated_text", "Error.")
                if not gemini_responses_for_vla["reasoning"] and api_reasoning_result.get("error"):
                     gemini_responses_for_vla["reasoning"] = api_reasoning_result.get("error", "Unknown reasoning error")
                
                if api_reasoning_result.get("full_response_content"):
                    current_chat_history.append(api_reasoning_result["full_response_content"])
                print(f"[AgentLoop] Reasoning result (first 50 chars): {gemini_responses_for_vla['reasoning'][:50] if gemini_responses_for_vla['reasoning'] else 'None'}")

                # --- Stage 3: Action ---
                print("[AgentLoop] Stage 3: Action")
                action_prompt = prompts.get("action", "")
                
                action_generation_config = types.GenerateContentConfig(
                    response_mime_type="application/json",
                    thinking_config=types.ThinkingConfig(thinking_budget=THINKING_BUDGET),
                    system_instruction=SYSTEM_PROMPT_ACTION
                )
                
                current_chat_history.append(types.UserContent(parts=[types.Part.from_text(text = action_prompt)]))
                
                print("[AgentLoop] Calling Gemini for Action...")
                api_action_result = await call_gemini_api_live(
                    stage="action",
                    contents_for_api=current_chat_history,
                    generation_config_override=action_generation_config
                )
                gemini_responses_for_vla["action"] = api_action_result.get("generated_text", "{}")
                if not gemini_responses_for_vla["action"] and api_action_result.get("error"):
                     gemini_responses_for_vla["action"] = json.dumps({"error": api_action_result.get("error", "Unknown action error")})
                print(f"[AgentLoop] Action result (first 50 chars): {gemini_responses_for_vla['action'][:50] if gemini_responses_for_vla['action'] else 'None'}")
                
                # --- Send annotation back to VLA server ---
                annotation_to_send = {
                    "responses": gemini_responses_for_vla,
                    "associatedImageTimestamp": timestamp
                }
                
                await websocket.send(json.dumps({
                    "messageId": f"gemini-synth-{os.urandom(4).hex()}",
                    "type": "userAnnotation", 
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                    "source": "geminiSynthesizerAgent",
                    "payload": annotation_to_send
                }))
                print(f"[AgentLoop] Synthesizer: Sent annotation for timestamp {timestamp}")

            elif message.get("type") == "setOperationMode":
                print(f"[AgentLoop] Received setOperationMode: {message.get('payload')}")
                pass

if __name__ == "__main__":
    print("[Main] Starting Gemini Data Synthesis Agent...")
    if not (genai and GEMINI_API_KEY and gemini_client): 
        print("[Main] FATAL: Gemini SDK not properly loaded, API Key not set, or Client not initialized. Please check configuration. Exiting.")
    else:
        try:
            asyncio.run(gemini_data_synthesis_agent())
        except KeyboardInterrupt:
            print("[Main] Gemini Data Synthesis Agent stopped by user.")
        except ConnectionRefusedError:
            print(f"[Main] Error: Connection to WebSocket server {WEBSOCKET_URI} refused. Is the VLA server running?")
        except Exception as e:
            print(f"[Main] An unexpected error occurred in synthesizer: {e}")
            print(traceback.format_exc())