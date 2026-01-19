import type {ConnectOptions} from "@/composable/interfaces/Inter2Robot.ts";

export function connectRobotWebSocket(
    onData: (data: any) => void,
    options: ConnectOptions = {}
): { socket: WebSocket | null; close: () => void } {
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    const path = options.path ?? '/api/ws/robot-data';
    const normalizedPath = path.startsWith('/') ? path : '/' + path;
    const url = `${protocol}//${location.host}${normalizedPath}`;

    let socket: WebSocket | null = null;

    try {
        socket = new WebSocket(url);
    } catch (err) {
        console.error('WebSocket create error:', err);
        return {
            socket: null,
            close: () => {
            }
        };
    }

    socket.onopen = (ev) => {
        options.onOpen?.(ev);
    };

    socket.onclose = (ev) => {
        options.onClose?.(ev);
    };

    socket.onerror = (ev) => {
        console.error('WebSocket error:', ev);
        options.onError?.(ev);
    };

    socket.onmessage = (event: MessageEvent) => {
        try {
            const res = JSON.parse(event.data);
            if (res.mode === 'BACKEND_INIT') return;
            const dataContent = res.payload || res;
            if (res.timestamp && dataContent.timestamp === undefined) {
                dataContent.timestamp = res.timestamp;
            }
            onData(dataContent);
        } catch (err) {
            console.error('WebSocket message parse error:', err);
        }
    };

    return {
        socket,
        close: () => {
            if (!socket) return;
            try {
                socket.close();
            } catch (e) {
                console.error('WebSocket close error:', e);
            }
        }
    };
}
