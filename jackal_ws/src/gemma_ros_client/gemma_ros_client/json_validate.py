import json
import jsonschema
import re

MY_CUSTOM_SCHEMA = {
    "type": "object",
    "properties": {
        "scene": {
            "type": "string",
            "description": "Current scene description based on image or context."
        },
        "action": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": "Name of the function or action to perform."
                },
                "parameters": {
                    "type": "object", # 매개변수는 객체(딕셔너리) 형태여야 함
                    "description": "Parameters required for the action, as a key-value map."
                }
            },
            "required": ["name", "parameters"], # action 객체는 name과 parameters를 필수로 가져야 함
            "description": "The determined action to take."
            # "additionalProperties": False
        },
        "reason": {
            "type": "string",
            "description": "Detailed explanation for the action decided."
        }
    },
    "required": ["scene", "action", "reason"], # 최상위 객체는 scene, action, reason을 필수로 가져야 함
    "description": "Custom AI decision output format schema."
    # "additionalProperties": False
}


# Markdown 코드 블록에서 JSON 문자열 추출 함수
def extract_json_from_markdown(text):
    """텍스트에서 ```json ... ``` 블록 안의 JSON 문자열을 추출합니다."""
    if not isinstance(text, str):
        return "" # 문자열이 아니면 빈 문자열 반환

    # 정규 표현식을 사용하여 ```json ... ``` 패턴 찾기
    # re.DOTALL은 .이 줄바꿈 문자도 포함하도록 함
    match = re.search(r'```json\s*\n(.*?)\n\s*```', text, re.DOTALL)

    if match:
        json_string = match.group(1).strip()
        # 추출된 문자열이 비어있지 않은지 확인
        if json_string:
            return json_string
        # 마커는 있지만 안에 내용이 없으면 원본 텍스트 반환 (혹시 마커 없이 JSON만 있는 경우 대비)
        return text.strip()
    else:
        # 마커가 없으면 입력 문자열 전체를 JSON으로 간주
        return text.strip()

# JSON 문법 유효성 검사 함수
def is_json_syntax_valid(json_string):
    """JSON 문자열의 문법 유효성만 검사합니다."""
    if not json_string:
        return False, "JSON 문자열이 비어 있습니다."
    try:
        json.loads(json_string)
        return True, None
    except json.JSONDecodeError as e:
        return False, f"JSON 문법 오류: {e}"
    except Exception as e:
         return False, f"알 수 없는 JSON 로딩 오류: {e}"


# JSON 구조/스키마 유효성 검사 함수
def is_json_structure_valid(data, schema):
    """Python 객체가 주어진 JSON 스키마에 맞는지 검사합니다."""
    try:
        jsonschema.validate(instance=data, schema=schema)
        return True, None
    except jsonschema.ValidationError as e:
        # 오류 경로와 메시지를 포함하여 상세 정보 반환
        return False, f"JSON 스키마 오류: {e.message} (경로: {'/'.join(map(str, e.path))})"
    except Exception as e:
         return False, f"스키마 검사 중 알 수 없는 오류 발생: {e}"


# 통합 유효성 검사 함수
def validate_custom_json(input_text):
    """
    입력 텍스트(Markdown 포함 가능)의 JSON이 정의된 커스텀 스키마에 맞는지 검사합니다.

    Args:
        input_text (str): JSON 문자열 또는 Markdown 코드 블록에 감싸진 JSON 문자열.

    Returns:
        tuple: (성공 여부(bool), 메시지(str), 파싱된 JSON 데이터(dict 또는 list) 또는 None)
               성공 시 (True, "성공 메시지", data), 실패 시 (False, "오류 메시지", None)
    """
    # 1단계: Markdown 블록 추출
    json_string = extract_json_from_markdown(input_text)

    # 2단계: JSON 문법 유효성 검사
    syntax_valid, error_message = is_json_syntax_valid(json_string)

    if not syntax_valid:
        return False, error_message, None # 문법 오류 시 바로 실패

    # 문법 유효 시 Python 객체로 로드
    try:
        json_data = json.loads(json_string)
    except Exception as e: # 이론적으로 is_json_syntax_valid에서 잡혔어야 하지만, 안전성 강화
         return False, f"JSON 데이터 로딩 중 오류 발생: {e}", None

    # 3단계: JSON 구조/스키마 유효성 검사
    structure_valid, error_message = is_json_structure_valid(json_data, MY_CUSTOM_SCHEMA)

    if not structure_valid:
        return False, error_message, json_data # 구조/스키마 오류 시 실패 (데이터 포함 반환 가능)

    # 모든 검사 통과
    return True, "JSON이 유효하고 스키마에 맞습니다.", json_data