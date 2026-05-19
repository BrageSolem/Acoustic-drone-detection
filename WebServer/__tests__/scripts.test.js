/**
 * Tests for client `scripts.js`.
 * Mocks DOM, Leaflet (L), navigator.geolocation and mediaDevices, and socket `io`.
 */

/**
 * @jest-environment jsdom
 */



beforeEach(() => {
    document.body.innerHTML = `
        <div id="map"></div>
        <div id="droneList"></div>
        <video id="droneVideo"></video>
        <svg>
          <circle id="movingDot" cx="0" cy="0"></circle>
          <line id="radiusLine" x2="0" y2="0"></line>
        </svg>
        <select id="debugDropdown">
          <option value="up">Up</option>
          <option value="right">Right</option>
          <option value="down">Down</option>
          <option value="left">Left</option>
        </select>
    `;

    const emitMock = jest.fn();
    const onMock = jest.fn();
    global.__mockSocketInstance = { emit: emitMock, on: onMock };
    global.__mockSocket = () => global.__mockSocketInstance;
    global.io = jest.fn(() => global.__mockSocketInstance);

    global.L = {
        map: jest.fn(() => ({ setView: jest.fn(), removeLayer: jest.fn() })),
        tileLayer: jest.fn(() => ({ addTo: jest.fn() })),
        marker: jest.fn(() => ({
            addTo: jest.fn(() => ({
                bindPopup: jest.fn(() => ({ openPopup: jest.fn() }))
            })),
            bindPopup: jest.fn(() => ({ openPopup: jest.fn() })),
        })),
        icon: jest.fn(() => ({}))
    };

    global.navigator.geolocation = {
        getCurrentPosition: jest.fn((success) => {
            success({ coords: { latitude: 12.34, longitude: 56.78, accuracy: 10 } });
        })
    };

    global.navigator.mediaDevices = {
        getUserMedia: jest.fn(() => Promise.resolve({}))
    };

    jest.resetModules();
});

test('getCurrentPosition resolves with lat/lng', async () => {
    const scripts = require('../public/scripts.js');
    const pos = await scripts.getCurrentPosition();
    expect(pos.lat).toBeCloseTo(12.34);
    expect(pos.lng).toBeCloseTo(56.78);
    expect(Number.isFinite(pos.accuracy)).toBe(true);
});

test('updateDiagram sets attributes on svg elements', () => {
    const { updateDiagram } = require('../public/scripts.js');
    updateDiagram(90); // 90 degrees
    const dot = document.getElementById('movingDot');
    const line = document.getElementById('radiusLine');
    expect(dot.getAttribute('cx')).not.toBeNull();
    expect(dot.getAttribute('cy')).not.toBeNull();
    expect(line.getAttribute('x2')).not.toBeNull();
    expect(line.getAttribute('y2')).not.toBeNull();
});

test('addDrone emits newDrone via socket', async () => {
    const scripts = require('../public/scripts.js');
    const newDrone = await scripts.addDrone();
    expect(global.__mockSocketInstance.emit).toHaveBeenCalledWith('newDrone', expect.objectContaining({
        id: expect.stringMatching(/^Drone_/),
        lat: expect.any(Number),
        lng: expect.any(Number),
        time: expect.any(String)
    }));
});
