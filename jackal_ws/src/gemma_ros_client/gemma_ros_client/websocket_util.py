# ros2_websocket_server_util.py

import asyncio
import websockets
import json
import uuid
from datetime import datetime, timezone
import logging
import threading
import time
from typing import Set, Callable, Any, Dict, Optional

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("ROS2WebSocketServer")

class ROS2WebSocketServer:
    """
    ROS2 노드에서 웹소켓 서버 역할을 수행하기 위한 유틸리티 클래스입니다.
    여러 클라이언트의 연결을 관리하고, 메시지를 주고받습니다.
    asyncio를 사용하여 비동기적으로 작동하며, 별도의 스레드에서 이벤트 루프를 실행합니다.
    """

    def __init__(self, host: str, port: int, server_name: str = "ros2_server_node"):
        self._host = host
        self._port = port
        self._server_name = server_name
        self._server_instance = None
        self._is_running = False
        self._loop = None
        self._thread = None
        self._connected_clients: Set[websockets.WebSocketServerProtocol] = set()

        # 콜백 함수들 (path 인자 제거)
        self._on_server_started_callback: Optional[Callable[[], None]] = None
        self._on_client_connected_callback: Optional[Callable[[websockets.WebSocketServerProtocol], None]] = None # path 제거
        self._on_client_disconnected_callback: Optional[Callable[[websockets.WebSocketServerProtocol, bool, int, str], None]] = None # path 제거
        self._on_message_callback: Optional[Callable[[websockets.WebSocketServerProtocol, Dict[str, Any]], None]] = None # path 제거
        self._on_error_callback: Optional[Callable[[Exception], None]] = None
        self._on_server_stopped_callback: Optional[Callable[[], None]] = None

    def register_callbacks(self, 
                           on_server_started: Optional[Callable[[], None]] = None,
                           on_client_connected: Optional[Callable[[websockets.WebSocketServerProtocol], None]] = None, # path 제거
                           on_client_disconnected: Optional[Callable[[websockets.WebSocketServerProtocol, bool, int, str], None]] = None, # path 제거
                           on_message: Optional[Callable[[websockets.WebSocketServerProtocol, Dict[str, Any]], None]] = None, # path 제거
                           on_error: Optional[Callable[[Exception], None]] = None,
                           on_server_stopped: Optional[Callable[[], None]] = None):
        if on_server_started: self._on_server_started_callback = on_server_started
        if on_client_connected: self._on_client_connected_callback = on_client_connected
        if on_client_disconnected: self._on_client_disconnected_callback = on_client_disconnected
        if on_message: self._on_message_callback = on_message
        if on_error: self._on_error_callback = on_error
        if on_server_stopped: self._on_server_stopped_callback = on_server_stopped
        logger.info("서버 콜백 함수들이 등록되었습니다.")

    def _generate_uuid(self) -> str:
        return str(uuid.uuid4())

    def _create_message(self, message_type: str, payload: Dict[str, Any]) -> str:
        message = {
            "messageId": self._generate_uuid(),
            "type": message_type,
            "timestamp": datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z'),
            "source": self._server_name,
            "payload": payload
        }
        return json.dumps(message)

    async def _client_handler(self, websocket: websockets.WebSocketServerProtocol):
        """
        개별 클라이언트 연결을 처리하는 핸들러입니다.
        path 정보는 이 핸들러에서 직접 사용하지 않습니다.
        """
        self._connected_clients.add(websocket)
        # path 정보가 없으므로 로깅에서 제거하거나 기본값 사용
        logger.info(f"클라이언트 연결됨: {websocket.remote_address} (총 클라이언트: {len(self._connected_clients)})")
        if self._on_client_connected_callback:
            try:
                self._on_client_connected_callback(websocket) # path 인자 없이 호출
            except Exception as e:
                logger.error(f"on_client_connected 콜백 실행 중 오류: {e}")

        try:
            async for raw_message in websocket:
                try:
                    message = json.loads(raw_message)
                    logger.debug(f"메시지 수신 ({websocket.remote_address}): {message}")
                    if self._on_message_callback:
                        try:
                            self._on_message_callback(websocket, message) # path 인자 없이 호출
                        except Exception as e:
                            logger.error(f"on_message 콜백 실행 중 오류: {e} (메시지: {message})")
                except json.JSONDecodeError:
                    logger.error(f"수신된 메시지 JSON 파싱 실패 ({websocket.remote_address}): {raw_message}")
                except Exception as e:
                    logger.error(f"메시지 처리 중 예기치 않은 오류 ({websocket.remote_address}): {e}")
        
        except websockets.exceptions.ConnectionClosed as e:
            logger.info(f"클라이언트 연결 종료됨 ({websocket.remote_address}): code={e.code}, reason='{e.reason}'")
            if self._on_client_disconnected_callback:
                try:
                    was_clean = e.code == 1000 or e.code == 1001
                    self._on_client_disconnected_callback(websocket, was_clean, e.code, e.reason or "") # path 인자 없이 호출
                except Exception as cb_e:
                    logger.error(f"on_client_disconnected 콜백 실행 중 오류: {cb_e}")
        
        except Exception as e:
            logger.error(f"클라이언트 핸들러 오류 ({websocket.remote_address}): {e}")
            if self._on_error_callback: 
                try:
                    self._on_error_callback(e)
                except Exception as cb_e:
                    logger.error(f"on_error 콜백 실행 중 오류 (client_handler): {cb_e}")
        finally:
            self._connected_clients.remove(websocket)
            logger.info(f"클라이언트 연결 정리됨: {websocket.remote_address} (남은 클라이언트: {len(self._connected_clients)})")

    async def _run_server(self):
        self._is_running = True
        try:
            logger.info(f"웹소켓 서버 시작 시도 중 (호스트: {self._host}, 포트: {self._port})...")
            async with websockets.serve(self._client_handler, self._host, self._port) as server:
                self._server_instance = server 
                logger.info(f"웹소켓 서버가 {self._host}:{self._port} 에서 성공적으로 시작되었습니다.")
                if self._on_server_started_callback:
                    try:
                        self._on_server_started_callback()
                    except Exception as e:
                        logger.error(f"on_server_started 콜백 실행 중 오류: {e}")
                
                while self._is_running:
                    await asyncio.sleep(0.1) 
                
                logger.info("서버 종료 신호 수신됨. 정리 시작...")
        except OSError as e: 
            logger.error(f"웹소켓 서버 시작 실패 (OSError): {e}")
            if self._on_error_callback:
                try:
                    self._on_error_callback(e)
                except Exception as cb_e:
                    logger.error(f"on_error 콜백 실행 중 오류 (run_server/OSError): {cb_e}")
            self._is_running = False 
        except Exception as e:
            logger.error(f"웹소켓 서버 실행 중 예기치 않은 오류: {e}")
            if self._on_error_callback:
                try:
                    self._on_error_callback(e)
                except Exception as cb_e:
                    logger.error(f"on_error 콜백 실행 중 오류 (run_server): {cb_e}")
            self._is_running = False 
        finally:
            if self._server_instance: 
                logger.info("웹소켓 서버 인스턴스 닫는 중...")
                self._server_instance.close()
                await self._server_instance.wait_closed() 
                logger.info("웹소켓 서버 인스턴스가 닫혔습니다.")
            
            if self._on_server_stopped_callback and not self._is_running : 
                try:
                    self._on_server_stopped_callback()
                except Exception as e:
                    logger.error(f"on_server_stopped 콜백 실행 중 오류: {e}")
            logger.info("웹소켓 서버 실행 루프가 종료되었습니다.")

    async def _send_to_client_async(self, client_ws: websockets.WebSocketServerProtocol, message_str: str):
        try:
            await client_ws.send(message_str)
            logger.debug(f"메시지 발신 성공 ({client_ws.remote_address}): {message_str}")
        except websockets.exceptions.ConnectionClosed:
            logger.warning(f"메시지 발신 실패: 클라이언트 연결 종료됨 ({client_ws.remote_address})")
        except Exception as e:
            logger.error(f"메시지 발신 중 오류 ({client_ws.remote_address}): {e}")

    def send_message_to_client(self, client_ws: websockets.WebSocketServerProtocol, message_type: str, payload: Dict[str, Any]):
        if not self._is_running or not self._loop or not self._loop.is_running():
            logger.error("웹소켓 서버가 실행 중이 아니거나 이벤트 루프가 없습니다. 메시지를 보낼 수 없습니다.")
            return
        if not client_ws:
            logger.warning("유효하지 않거나 이미 닫힌 클라이언트입니다. 메시지를 보낼 수 없습니다.")
            return
        message_str = self._create_message(message_type, payload)
        asyncio.run_coroutine_threadsafe(self._send_to_client_async(client_ws, message_str), self._loop)

    def broadcast_message(self, message_type: str, payload: Dict[str, Any]):
        if not self._is_running or not self._loop or not self._loop.is_running():
            logger.error("웹소켓 서버가 실행 중이 아니거나 이벤트 루프가 없습니다. 브로드캐스트할 수 없습니다.")
            return
        if not self._connected_clients:
            logger.info("연결된 클라이언트가 없어 브로드캐스트하지 않습니다.")
            return
        message_str = self._create_message(message_type, payload)
        logger.info(f"브로드캐스트 메시지 발신 시도 (클라이언트 {len(self._connected_clients)}명): {message_type}")
        clients_to_send = list(self._connected_clients)
        for client_ws in clients_to_send:
            if client_ws:
                 asyncio.run_coroutine_threadsafe(self._send_to_client_async(client_ws, message_str), self._loop)
            else:
                logger.warning(f"브로드캐스트 중 닫힌 클라이언트 발견: {client_ws.remote_address if client_ws else 'N/A'}")

    def start(self):
        if self._is_running:
            logger.warning("서버가 이미 실행 중입니다.")
            return
        logger.info("웹소켓 서버를 시작합니다...")
        self._loop = asyncio.new_event_loop()
        def run_loop(loop):
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self._run_server())
            except Exception as e: 
                logger.critical(f"이벤트 루프 실행 중 심각한 오류 발생: {e}", exc_info=True)
            finally:
                if not loop.is_closed(): 
                    loop.close() 
                logger.info("이벤트 루프가 닫혔습니다.")
        self._thread = threading.Thread(target=run_loop, args=(self._loop,), daemon=True)
        self._thread.start()
        logger.info("웹소켓 서버 스레드가 시작되었습니다.")

    def stop(self):
        logger.info("웹소켓 서버를 중지합니다...")
        if not self._is_running:
            logger.info("서버가 이미 중지되었거나 시작되지 않았습니다.")
            return
        self._is_running = False 
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
            logger.info("이벤트 루프 중지 요청됨.")
        if self._thread and self._thread.is_alive():
            logger.info("웹소켓 서버 스레드 조인 대기 중...")
            self._thread.join(timeout=10) 
            if self._thread.is_alive():
                logger.warning("웹소켓 서버 스레드가 시간 내에 종료되지 않았습니다.")
            else:
                logger.info("웹소켓 서버 스레드가 성공적으로 조인되었습니다.")
        self._loop = None 
        self._thread = None
        self._server_instance = None
        logger.info("웹소켓 서버가 중지되었습니다.")

    def get_connected_clients_count(self) -> int:
        return len(self._connected_clients)