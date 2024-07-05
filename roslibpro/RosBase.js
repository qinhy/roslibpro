export class RosBase {
    constructor(rosip, topic_name, topic_type, rate = 10, is_service = false) {
        this.rosip = rosip;
        this.topic_name = topic_name;
        this.topic_type = topic_type;
        this.rate = rate;
        this.self_closed = false;
        this.connected_cnt = 0;
        this.is_service = is_service;
        this.ros = null;
    }

    connectROS(callback = {
        onError: () => { },
        onConnection: () => { },
        onClose: () => { }
    }, rate = this.rate, call_from_outter = true) {
        if (this.isConnectDone() || this.isConnecting() || this.isClosing()) return false;

        if (call_from_outter) {
            this.connected_cnt = 0;
            this.self_closed = false;
        }

        try {
            this.ros = new ROSLIB.Ros({
                url: `ws://${this.rosip}`
            });

            this.ros.on("error", error => {
                this.ros = null;
                if (call_from_outter) this.connected_cnt++;
                callback.onError();
            });

            this.ros.on("connection", () => {
                this.self_closed = false;
                if (call_from_outter) this.connected_cnt++;
                callback.onConnection();
            });

            this.ros.on("close", () => {
                this.ros = null;
                callback.onClose();
            });

            this.topic = this.is_service
                ? new ROSLIB.Service({
                    ros: this.ros,
                    name: this.topic_name,
                    serviceType: this.topic_type
                })
                : new ROSLIB.Topic({
                    ros: this.ros,
                    name: this.topic_name,
                    messageType: this.topic_type,
                    rate: rate
                });
        } catch (e) {
            return false;
        }
        return true;
    }

    isNULL() {
        return this.ros === null || this.ros.socket === null;
    }

    isConnectDone() {
        return !this.isNULL() && this.ros.socket.readyState === WebSocket.OPEN;
    }

    isConnecting() {
        return !this.isNULL() && this.ros.socket.readyState === WebSocket.CONNECTING;
    }

    isClosing() {
        return !this.isNULL() && this.ros.socket.readyState === WebSocket.CLOSING;
    }

    isClosed() {
        return this.isNULL() || this.ros.socket.readyState === WebSocket.CLOSED;
    }

    close(is_self_closed = true) {
        this.self_closed = is_self_closed;
        if (this.ros && this.ros.socket && this.ros.socket.readyState !== WebSocket.CLOSED) {
            this.ros.close();
        }
        this.ros = null;
    }
}
