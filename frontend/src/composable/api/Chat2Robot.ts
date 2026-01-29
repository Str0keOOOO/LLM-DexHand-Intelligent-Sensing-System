import request from '@/composable/utils/request'
import type {ControlCommand} from '@/composable/interfaces/Inter2Robot';

export function sendControlCommand(data: ControlCommand) {
    return request.post<{ status: string; sent_to: string }>('/ros_ws/control', data) as unknown as Promise<{
        status: string;
        sent_to: string
    }>;
}

