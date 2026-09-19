"""
Framework CLI Entry Point.
Run with: python -m backend.main --backend gemini --drone matlab
"""
import argparse
import uvicorn
from backend.api.server import create_app
from backend.core.config import settings
from backend.core.logger import get_logger

logger = get_logger("Main")


def main():
    parser = argparse.ArgumentParser(description="LLM-VLM Adaptive Drone Framework Server")
    parser.add_argument("--backend", type=str, default="gemini",
                        choices=["gemini", "qwen", "qwen25", "local"],
                        help="LLM/VLM Model Backend. "
                             "'qwen'/'qwen25'/'local' = offline split (Qwen2-VL-2B for VLM + Qwen2.5-3B for planning). "
                             "'gemini' = Gemini Flash API.")
    parser.add_argument("--drone", type=str, default="sim", choices=["sim", "real", "matlab", "pysimverse"], help="Drone Execution Target (sim | real | matlab | pysimverse)")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Host IP address")
    parser.add_argument("--port", type=int, default=8000, help="Port number")

    args = parser.parse_args()

    logger.info(f"Starting Framework Backend [Model: {args.backend.upper()}, Target: {args.drone.upper()}]...")
    if args.backend.lower() == "gemini":
        key = settings.gemini_api_key
        if not key or key.lower() in {"your_key_here", "changeme", "paste_here"}:
            logger.warning("No GEMINI_API_KEY in Project_phase1/.env. Planner will use the offline rule parser.")
            logger.warning("Put the key in .env (not only .env.example). Get one at https://aistudio.google.com/apikey")
        else:
            logger.info(f"GEMINI_API_KEY loaded from .env ({len(key)} chars).")
    app = create_app(drone_mode=args.drone, model_mode=args.backend)

    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
