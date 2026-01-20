export interface ConnectOptions {
    path?: string;
    onOpen?: (ev: Event) => void;
    onClose?: (ev: CloseEvent) => void;
    onError?: (ev: Event) => void;
}

export interface RosHealth {
    status: 'ok';
    ros_bridge?: 'connected' | 'initializing';
}