export interface ConnectOptions {
    path?: string;
    onOpen?: (ev: Event) => void;
    onClose?: (ev: CloseEvent) => void;
    onError?: (ev: Event) => void;
}