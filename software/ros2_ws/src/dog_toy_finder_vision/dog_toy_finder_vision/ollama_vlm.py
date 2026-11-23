"""Ollama chat requests shared by the live node and the offline replay tool.

Keeping this free of ROS imports means a saved capture can be replayed outside
a running graph with exactly the same request body, model-unload policy, and
answer normalisation used live.
"""

import base64
import json
import time
import urllib.request

from . import vision_answer


def ask_model(model, jpegs, target, prompt, chat_url,
              compare_models=False, timeout=180, config=None):
    """Ask one Ollama vision model about JPEG bytes for a rectified pair."""
    images_b64 = [base64.b64encode(jpeg).decode() for jpeg in jpegs]
    body = {
        "model": model,
        "stream": False,
        "think": False,
        "format": vision_answer.ANSWER_SCHEMA,
        "keep_alive": 0 if compare_models else "30m",
        "options": {"temperature": 0, "num_ctx": 8192},
        "messages": [{"role": "user", "content": prompt,
                      "images": images_b64}],
    }
    start = time.time()
    try:
        request = urllib.request.Request(
            chat_url, data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=timeout) as reply:
            answer = json.loads(json.load(reply)["message"]["content"])
        answer = vision_answer.tidy(
            {key: answer[key] for key in vision_answer.ANSWER_SCHEMA["required"]
             if key in answer}, config)
    except Exception as error:
        return {"model": model, "method": "ml", "error":
                f"Vision model failed: {error}",
                "seconds": round(time.time() - start, 2)}
    answer["model"] = model
    answer["method"] = "ml"
    answer["distance_source"] = "vlm_estimate"
    answer["seconds"] = round(time.time() - start, 2)
    return answer


def unload_model(model, generate_url, timeout=15, logger=None):
    """Ask Ollama to evict a model so the next one gets all the VRAM."""
    body = {"model": model, "keep_alive": 0}
    try:
        request = urllib.request.Request(
            generate_url, data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=timeout) as reply:
            reply.read()
        return True
    except Exception as error:
        if logger is not None:
            logger.warning(
                f"Could not unload {model}; it may still hold VRAM: {error}")
        return False
