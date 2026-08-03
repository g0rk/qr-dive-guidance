import websockets
import asyncio
import json

from target_stream import target_stream   # ← tek ekleme

async def websocket_handler(command_queue):
    uri = "ws://127.0.0.1:8765"
    print(f"Connecting: {uri}")

    try:
        async with websockets.connect(uri) as ws:
            print("Connected.")

            async def receive_loop():
                async for message in ws:
                    try:
                        data = json.loads(message)
                        print(f"{data}")
                    except json.JSONDecodeError:
                        print(message)

            async def send_loop():
                while True:
                    command = await command_queue.get()
                    payload = {"command": command}
                    await ws.send(json.dumps(payload))
                    print(f"{payload}")

            target_queue: asyncio.Queue = asyncio.Queue(maxsize=5)

            async def target_forward_loop():
                while True:
                    payload = await target_queue.get()
                    await ws.send(json.dumps(payload))

            await asyncio.gather(
                receive_loop(),
                send_loop(),
                target_stream(target_queue, hz=10.0),   # ← hedef araç
                target_forward_loop(),
            )

    except websockets.ConnectionClosed:
        print("Disconnected.")
    except Exception as e:
        print(f"Connection error: {e}")

async def input_loop(command_queue):
    while True:
        command = await asyncio.to_thread(input, "> ")

        if command.lower() in ("exit", "quit"):
            break

        if command.strip():
            await command_queue.put(command.strip())

async def main():
    command_queue = asyncio.Queue()

    ws_task = asyncio.create_task(websocket_handler(command_queue))

    try:
        await input_loop(command_queue)
    finally:
        ws_task.cancel()
        try:
            await ws_task
        except asyncio.CancelledError:
            pass

if __name__ == "__main__":
    asyncio.run(main())