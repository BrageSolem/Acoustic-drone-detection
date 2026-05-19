/**
 * @jest-environment node
 */

const request = require('supertest');
const ioClient = require('socket.io-client');

jest.setTimeout(10000);

describe('server', () => {
    let server;
    let port;

    beforeAll((done) => {
        const srvModule = require('../server');
        server = srvModule.server;

        // If the server is already listening (e.g. started elsewhere), just use its port.
        if (server.listening) {
            port = server.address().port;
            done();
            return;
        }

        // listen on ephemeral port
        server.listen(0, () => {
            port = server.address().port;
            done();
        });
    });

    afterAll((done) => {
        // Only close if the server is currently listening.
        if (server && server.listening) {
            server.close(done);
        } else {
            done();
        }
    });

    afterAll((done) => {
        server.close(done);
    });

    test('GET /health returns status ok and pi', async () => {
        const res = await request(server).get('/health');
        expect(res.status).toBe(200);
        expect(res.body.status).toBe('ok');
        expect(res.body.pi).toBeDefined();
    });

    test('socket: newDrone triggers updateDrones broadcast', (done) => {
        const clientA = ioClient.connect(`http://127.0.0.1:${port}`, { transports: ['websocket'], forceNew: true });
        const clientB = ioClient.connect(`http://127.0.0.1:${port}`, { transports: ['websocket'], forceNew: true });

        clientA.on('connect', () => {
            clientB.on('connect', () => {
                clientA.on('updateDrones', (drones) => {
                    try {
                        expect(Array.isArray(drones)).toBe(true);
                        clientA.disconnect();
                        clientB.disconnect();
                        done();
                    } catch (err) {
                        clientA.disconnect();
                        clientB.disconnect();
                        done(err);
                    }
                });

                // Emit from B; server should broadcast update to A (and B)
                clientB.emit('newDrone', { id: 'test-socket', lat: 1, lng: 2, time: 't' });
            });
        });

        // safety timeout
        setTimeout(() => {
            clientA.disconnect();
            clientB.disconnect();
            done(new Error('timeout waiting for updateDrones'));
        }, 3000);
    });
});
