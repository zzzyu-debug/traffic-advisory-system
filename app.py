"""
中国交通模拟系统 — Flask 后端服务
调用 C 可执行文件 main_system2.exe 完成路径计算
"""
import json
import os
import subprocess
import sys
import tempfile
from flask import Flask, request, jsonify, send_from_directory

app = Flask(__name__, static_folder="static", static_url_path="")

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
EXE = os.path.join(BASE_DIR, "main_system2.exe" if sys.platform == "win32" else "main_system2")

# 缓存站点和城市列表（启动时加载）
_stations_cache = None
_cities_cache = None


def call_exe(*args):
    """调用 C 可执行文件，返回 stdout 内容（UTF-8）"""
    cmd = [EXE] + list(args)
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            timeout=120,
            cwd=BASE_DIR,
        )
        return result.stdout.decode("utf-8").strip()
    except subprocess.TimeoutExpired:
        return '{"status":"error","message":"查询超时，请重试"}'
    except Exception as e:
        return f'{{"status":"error","message":"{str(e)}"}}'


def load_cache():
    """启动时预加载站点和城市列表"""
    global _stations_cache, _cities_cache

    print("Loading cities...", flush=True)
    raw = call_exe("--list-cities")
    try:
        _cities_cache = json.loads(raw)
        print(f"  Loaded {len(_cities_cache.get('cities', []))} cities")
    except json.JSONDecodeError:
        print(f"  Failed to parse cities: {raw[:200]}")
        _cities_cache = {"cities": []}

    print("Loading stations...", flush=True)
    raw = call_exe("--list-stations")
    try:
        _stations_cache = json.loads(raw)
        print(f"  Loaded {len(_stations_cache.get('stations', []))} stations")
    except json.JSONDecodeError:
        print(f"  Failed to parse stations: {raw[:200]}")
        _stations_cache = {"stations": []}


# ===== API 路由 =====

@app.route("/api/stations")
def api_stations():
    """返回所有站点（含坐标）"""
    if _stations_cache is None:
        return jsonify({"stations": []})
    return jsonify(_stations_cache)


@app.route("/api/cities")
def api_cities():
    """返回所有城市（含坐标）"""
    if _cities_cache is None:
        return jsonify({"cities": []})
    return jsonify(_cities_cache)


@app.route("/api/query", methods=["POST"])
def api_query():
    """执行路径查询（通过 UTF-8 临时文件传递参数）"""
    data = request.get_json(force=True)
    from_name = data.get("from", "").strip()
    to_name = data.get("to", "").strip()
    mode = data.get("mode", 1)           # 1=最快 2=最省钱 3=最少换乘
    transport = data.get("transport", 1) # 1=火车 2=飞机

    if not from_name or not to_name:
        return jsonify({"status": "error", "message": "请输入起点和终点"})

    # 写入 UTF-8 临时文件（避免命令行编码问题）
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".txt", delete=False,
        encoding="utf-8", dir=BASE_DIR
    )
    try:
        tmp.write(f"{from_name}\n{to_name}\n{mode}\n{transport}\n")
        tmp.close()

        raw = call_exe("--query-file", tmp.name)
        result = json.loads(raw)
    except json.JSONDecodeError:
        result = {"status": "error", "message": f"后端解析失败: {raw[:200]}"}
    finally:
        try:
            os.unlink(tmp.name)
        except OSError:
            pass

    return jsonify(result)


# ===== 静态文件服务 =====

@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


# ===== 启动 =====
if __name__ == "__main__":
    print("=" * 50)
    print("  Traffic Advisory System — Web Server")
    print("=" * 50)
    print()
    print("Initializing data (may take 10-30 seconds)...")
    print()

    load_cache()

    print()
    port = int(os.environ.get("PORT", 5000))
    print(f"Server ready: http://localhost:{port}")
    print("Press Ctrl+C to stop")
    print()

    app.run(host="0.0.0.0", port=port, debug=False)
