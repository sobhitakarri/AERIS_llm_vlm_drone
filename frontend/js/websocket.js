/**
 * WebSocket Telemetry Consumer module.
 */
class TelemetrySocket {
    constructor(url, onMessage, onStatusChange) {
        this.url = url;
        this.onMessage = onMessage;
        this.onStatusChange = onStatusChange;
        this.ws = null;
        this.connect();
    }

    connect() {
        const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = this.url || `${wsProtocol}//${window.location.hostname || '127.0.0.1'}:8000/ws/telemetry`;

        try {
            this.ws = new WebSocket(wsUrl);

            this.ws.onopen = () => {
                if (this.onStatusChange) this.onStatusChange(true);
            };

            this.ws.onmessage = (event) => {
                const data = JSON.parse(event.data);
                if (this.onMessage) this.onMessage(data);
            };

            this.ws.onclose = () => {
                if (self.onStatusChange) this.onStatusChange(false);
                setTimeout(() => this.connect(), 2000);
            };

            this.ws.onerror = (err) => {
                if (this.onStatusChange) this.onStatusChange(false);
            };
        } catch (e) {
            if (this.onStatusChange) this.onStatusChange(false);
        }
    }
}
