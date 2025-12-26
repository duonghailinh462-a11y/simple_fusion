import socket
import struct
import json
import os
import sys
radar_read_path = os.path.join(os.path.dirname(__file__), '..', 'radar', 'radar_read')
sys.path.insert(0, radar_read_path)
import radar_pb2
import threading
import multiprocessing
import time
from google.protobuf.json_format import MessageToDict

# --- 配置区域 ---
LISTEN_PORT = 12400
SAVE_FILENAME = "radar_data.jsonl" 

file_write_lock = threading.Lock()

# 全局队列（由main.py传入）
data_queue = None

def read_exactly(sock, num_bytes):
    data = b''
    while len(data) < num_bytes:
        try:
            packet = sock.recv(num_bytes - len(data))
            if not packet: return None
            data += packet
        except Exception:
            return None
    return data

def handle_connection(conn, addr):
    print(f"--- [新连接] 设备 {addr[0]} 已接入 ---")
    try:
        while True:
            # 1. 读头
            head_data = read_exactly(conn, 8)
            if not head_data: break
            
            if head_data[:4] != b'\xAA\xAB\xAC\xAD':
                continue

            total_len = struct.unpack('<I', head_data[4:8])[0]
            
            # 2. 读体
            body_data = read_exactly(conn, total_len)
            if not body_data: break

            # 3. 解码并保存
            if len(body_data) > 36:
                # 记录数据到达设备的时间戳（毫秒）
                arrival_time_ms = int(time.time() * 1000)
                
                proto_content = body_data[30:-6]
                try:
                    obj = radar_pb2.ObjLocus()
                    obj.ParseFromString(proto_content)
                    # 第一步：先把 Protobuf 转成字典
                    raw_dict = MessageToDict(obj, preserving_proto_field_name=True)
                    
                    # 第二步：创建一个新字典，把 IP 和到达时间放在最前面
                    final_dict = {
                        "source_ip": addr[0],              # 这个会排在最前面
                        "arrival_time_ms": arrival_time_ms,  # 数据到达设备的时间戳
                        **raw_dict                         # 剩下的数据排在后面
                    }
                    
                    # 计算时间差（如果原数据中有 time 字段）
                    if 'time' in raw_dict:
                        try:
                            radar_send_time = int(raw_dict.get('time'))
                            time_diff_ms = arrival_time_ms - radar_send_time
                            final_dict['time_diff_ms'] = time_diff_ms  # 到达时间 - 发送时间
                        except (ValueError, TypeError):
                            final_dict['time_diff_ms'] = None  # 无法转换则设为 None
                    
                    # 保存到文件
                    with file_write_lock:
                        with open(SAVE_FILENAME, "a", encoding="utf-8") as f:
                            f.write(json.dumps(final_dict, ensure_ascii=False) + "\n")
                    
                    # 发送到队列（如果存在）
                    if data_queue is not None:
                        try:
                            data_queue.put(final_dict, block=False)
                        except Exception as e:
                            print(f"[{addr[0]}] 队列写入失败: {e}")
                    
                    current_time = final_dict.get('time', 'NoTime')
                    time_diff = final_dict.get('time_diff_ms', 'N/A')
                    print(f"[{addr[0]}] {current_time} 存入成功 | 时间差: {time_diff}ms")
                    
                except Exception as e:
                    print(f"[{addr[0]}] 解码错误: {e}")

    except Exception as e:
        print(f"[{addr[0]}] 连接异常: {e}")
    finally:
        conn.close()
        print(f"--- [断开] 设备 {addr[0]} 已下线 ---")

def run_server(queue=None):
    """
    运行雷达TCP服务器
    
    Args:
        queue: multiprocessing.Queue 用于传递数据到main.py（可选）
    """
    global data_queue
    data_queue = queue
    
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind(('0.0.0.0', LISTEN_PORT))
        server.listen(10)
        print(f">>> 多线程服务已启动，数据保存至 {SAVE_FILENAME} <<<")
        
        while True:
            conn, addr = server.accept()
            t = threading.Thread(target=handle_connection, args=(conn, addr))
            t.daemon = True
            t.start()
            
    except KeyboardInterrupt:
        print("\n停止服务")
    finally:
        server.close()

if __name__ == '__main__':
    run_server()