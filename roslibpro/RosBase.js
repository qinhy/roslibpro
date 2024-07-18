export class RosBase {
    constructor(rosip, topic_name, topic_type, rate = 10, is_service = false) {
        this.rosip = rosip;
        this.topic_name = topic_name;
        this.topic_type = topic_type;
        this.rate = rate;
        this.self_closed = false;
        this.connected_cnt = 0;
        this.is_service = is_service;
        this.getROS = ()=>null;
        this.topic = ()=>null;
    }

    connectROS(callback = {
        onError: (error) => { },
        onConnection: () => { },
        onClose: () => { }
    }, rate = this.rate, call_from_outter = true) {
        if (this.isConnectDone() || this.isConnecting() || this.isClosing()) return false;

        if (call_from_outter) {
            this.connected_cnt = 0;
            this.self_closed = false;
        }

        try {
            const ros = new ROSLIB.Ros({
                url: `ws://${this.rosip}`
            });
            this.getROS = ()=>ros;

            this.getROS().on("error", error => {
                
        this.getROS = ()=>null;
                if (call_from_outter) this.connected_cnt++;
                callback.onError(error);
            });

            this.getROS().on("connection", () => {
                this.self_closed = false;
                if (call_from_outter) this.connected_cnt++;
                callback.onConnection();
            });

            this.getROS().on("close", () => {
                
        this.getROS = ()=>null;
                callback.onClose();
            });

            const topic = this.is_service
                ? new ROSLIB.Service({
                    ros: this.getROS(),
                    name: this.topic_name,
                    serviceType: this.topic_type
                })
                : new ROSLIB.Topic({
                    ros: this.getROS(),
                    name: this.topic_name,
                    messageType: this.topic_type,
                    rate: rate
                });
            this.topic = ()=>topic;
        } catch (e) {
            return false;
        }
        return true;
    }

    isNULL() {
        return this.getROS() === null || this.getROS().socket === null;
    }

    isConnectDone() {
        return !this.isNULL() && this.getROS().socket.readyState === WebSocket.OPEN;
    }

    isConnecting() {
        return !this.isNULL() && this.getROS().socket.readyState === WebSocket.CONNECTING;
    }

    isClosing() {
        return !this.isNULL() && this.getROS().socket.readyState === WebSocket.CLOSING;
    }

    isClosed() {
        return this.isNULL() || this.getROS().socket.readyState === WebSocket.CLOSED;
    }

    close(is_self_closed = true) {
        this.self_closed = is_self_closed;
        if (this.getROS() && this.getROS().socket && this.getROS().socket.readyState !== WebSocket.CLOSED) {
            this.getROS().close();
        }
        
        this.getROS = ()=>null;
    }
}
