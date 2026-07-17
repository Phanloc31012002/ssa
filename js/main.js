import * as THREE from 'three';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { LightProbeGrid } from 'three/addons/lighting/LightProbeGrid.js';
import { LightProbeGridHelper } from 'three/addons/helpers/LightProbeGridHelper.js';
import { createWorldSettings, createWorld, addBroadphaseLayer, addObjectLayer, enableCollision, registerAll, updateWorld, rigidBody, box, MotionType } from 'crashcat';
import { Vehicle, MAX_SPEED } from './Vehicle.js';
import { Camera } from './Camera.js';
import { Controls } from './Controls.js';
import { AutoDriver } from './AutoDriver.js';
import { buildTrack, decodeCells, computeSpawnPosition, computeTrackBounds } from './Track.js';
import { buildWallColliders, createSphereBody } from './Physics.js';
import { SmokeTrails } from './Particles.js';
import { DriftMarks } from './DriftMarks.js';
import { GameAudio } from './Audio.js';
import { LapTimer } from './LapTimer.js';
import { ColorMapGLTFLoader } from './Loader.js';


const renderer = new THREE.WebGLRenderer( { antialias: true, outputBufferType: THREE.HalfFloatType } );
renderer.setSize( window.innerWidth, window.innerHeight );
renderer.setPixelRatio( window.devicePixelRatio );
renderer.shadowMap.enabled = true;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.0;

const bloomPass = new UnrealBloomPass( new THREE.Vector2( window.innerWidth, window.innerHeight ) );
bloomPass.strength = 0.02;
bloomPass.radius = 0.02;
bloomPass.threshold = 0.5;

renderer.setEffects( [ bloomPass ] );

document.body.appendChild( renderer.domElement );

const scene = new THREE.Scene();
scene.background = new THREE.Color( 0xadb2ba );
scene.fog = new THREE.Fog( 0xadb2ba, 30, 55 );

const dirLight = new THREE.DirectionalLight( 0xffffff, 3 );
dirLight.position.set( 11.4, 15, -5.3 );
dirLight.castShadow = true;
dirLight.shadow.mapSize.setScalar( 4096 );
dirLight.shadow.camera.near = 0.5;
dirLight.shadow.camera.far = 60;
dirLight.shadow.radius = 4;
scene.add( dirLight );

const hemiLight = new THREE.HemisphereLight( 0xc8d8e8, 0x7a8a5a, 2 );
hemiLight.position.copy( dirLight.position )
scene.add( hemiLight );


window.addEventListener( 'resize', () => {

	renderer.setSize( window.innerWidth, window.innerHeight );

} );

const loader = new ColorMapGLTFLoader();

const modelNames = [
	'vehicle-truck-yellow', 'vehicle-truck-green', 'vehicle-truck-purple', 'vehicle-truck-red',
	'track-straight', 'track-corner', 'track-bump', 'track-finish',
	'decoration-empty', 'decoration-forest', 'decoration-tents',
];

const models = {};

async function loadModels() {

	const promises = modelNames.map( ( name ) =>
		new Promise( ( resolve, reject ) => {

			loader.load( `models/${ name }.glb`, ( gltf ) => {

				const meshes = [];
				gltf.scene.traverse( ( child ) => {

					if ( child.isMesh ) {

						child.material.side = THREE.FrontSide;
						meshes.push( child );

					}

				} );

				// Godot imports vehicle models at root_scale=0.5
				if ( name.startsWith( 'vehicle-' ) ) {

					gltf.scene.scale.setScalar( 0.5 );

				}

				if ( meshes.length === 1 ) {

					const mesh = meshes[ 0 ];
					mesh.removeFromParent();
					models[ name ] = mesh;

				} else {

					models[ name ] = gltf.scene;

				}

				resolve();

			}, undefined, reject );

		} )
	);

	await Promise.all( promises );

}

async function init() {

	registerAll();
	await loadModels();

	const mapParam = new URLSearchParams( window.location.search ).get( 'map' );
	let customCells = null;
	let spawn = null;

	if ( mapParam ) {

		try {

			customCells = decodeCells( mapParam );
			spawn = computeSpawnPosition( customCells );

		} catch ( e ) {

			console.warn( 'Invalid map parameter, using default track' );

		}

	}

	// Compute track bounds and size physics/shadows to fit
	const bounds = computeTrackBounds( customCells );
	const hw = bounds.halfWidth;
	const hd = bounds.halfDepth;
	const groundSize = Math.max( hw, hd ) * 2 + 20;

	const shadowExtent = Math.max( hw, hd ) + 10;
	dirLight.shadow.camera.left = - shadowExtent;
	dirLight.shadow.camera.right = shadowExtent;
	dirLight.shadow.camera.top = shadowExtent;
	dirLight.shadow.camera.bottom = - shadowExtent;
	dirLight.shadow.camera.updateProjectionMatrix();

	scene.fog.near = groundSize * 0.4;
	scene.fog.far = groundSize * 0.8;

	buildTrack( scene, models, customCells );

	// Probes

	const probeHeight = 6;
	const probes = new LightProbeGrid(
		hw * 2, probeHeight, hd * 2,
		Math.max( 4, Math.round( hw / 4 ) ),
		2,
		Math.max( 4, Math.round( hd / 4 ) ),
	);
	probes.position.set( bounds.centerX, probeHeight / 2, bounds.centerZ );
	probes.bake( renderer, scene, { cubemapSize: 32, near: 0.1, far: groundSize } );
	scene.add( probes );

	// scene.add( new LightProbeGridHelper( probes, 0.5 ) );

	//

	const worldSettings = createWorldSettings();
	worldSettings.gravity = [ 0, - 9.81, 0 ];

	const BPL_MOVING = addBroadphaseLayer( worldSettings );
	const BPL_STATIC = addBroadphaseLayer( worldSettings );
	const OL_MOVING = addObjectLayer( worldSettings, BPL_MOVING );
	const OL_STATIC = addObjectLayer( worldSettings, BPL_STATIC );

	enableCollision( worldSettings, OL_MOVING, OL_STATIC );
	enableCollision( worldSettings, OL_MOVING, OL_MOVING );

	const world = createWorld( worldSettings );
	world._OL_MOVING = OL_MOVING;
	world._OL_STATIC = OL_STATIC;

	buildWallColliders( world, null, customCells );

	const roadHalf = groundSize / 2;
	rigidBody.create( world, {
		shape: box.create( { halfExtents: [ roadHalf, 0.01, roadHalf ] } ),
		motionType: MotionType.STATIC,
		objectLayer: OL_STATIC,
		position: [ bounds.centerX, - 0.125, bounds.centerZ ],
		friction: 5.0,
		restitution: 0.0,
	} );

	const sphereBody = createSphereBody( world, spawn ? spawn.position : null );

	const vehicle = new Vehicle();
	vehicle.rigidBody = sphereBody;
	vehicle.physicsWorld = world;

	if ( spawn ) {

		const [ sx, sy, sz ] = spawn.position;
		vehicle.spherePos.set( sx, sy, sz );
		vehicle.prevModelPos.set( sx, 0, sz );
		vehicle.container.rotation.y = spawn.angle;

	}

	const vehicleGroup = vehicle.init( models[ 'vehicle-truck-yellow' ] );
	scene.add( vehicleGroup );

	dirLight.target = vehicleGroup;

	const cam = new Camera();
	scene.add( cam.debug );

	const controls = new Controls();

	const autoDriver = new AutoDriver( customCells, scene );
	let autoPilot = autoDriver.enabled;   // bật mặc định nếu dựng được đường tâm

	const autoEl = document.createElement( 'div' );
	autoEl.style.cssText = 'position:absolute;top:12px;right:12px;color:#fff;font:600 12px -apple-system,sans-serif;background:rgba(0,0,0,0.5);padding:8px 12px;border-radius:8px;z-index:10;pointer-events:none;backdrop-filter:blur(8px);letter-spacing:0.06em;text-align:right;line-height:1.5;';
	document.body.appendChild( autoEl );

	function refreshAutoUI() {

		if ( ! autoDriver.enabled ) {

			autoEl.innerHTML = '<span style="color:#ff6e6e">AUTOPILOT N/A</span>';
			return;

		}

		const state = autoPilot
			? '<span style="color:#5af168">AUTOPILOT ON</span>'
			: 'AUTOPILOT OFF';

		autoEl.innerHTML = state +
			'<div style="opacity:0.55;font-weight:500;font-size:11px">P bật/tắt · O đường đua · G chỉnh</div>';

	}

	refreshAutoUI();

	// ── Bảng tinh chỉnh (phím G) ──
	const TUNABLES = [
		[ 'cornerSafety',    0.4, 1.2,  0.05, 'Tốc độ vào cua (cao = liều)' ],
		[ 'steerGain',       1.5, 8.0,  0.25, 'Độ mạnh bẻ lái' ],
		[ 'counterSteer',    0.0, 3.0,  0.1,  'Lái ngược khi trượt' ],
		[ 'driftSlip',       0.1, 0.8,  0.05, 'Ngưỡng tính là drift' ],
		[ 'throttleOnDrift', 0.0, 1.0,  0.05, 'Giữ ga khi drift' ],
		[ 'lookTime',        0.1, 1.0,  0.05, 'Nhìn trước (giây)' ],
		[ 'brakeGain',       1.0, 8.0,  0.5,  'Độ gắt khi phanh' ],
	];

	const panel = document.createElement( 'div' );
	panel.style.cssText = 'position:absolute;top:70px;right:12px;color:#fff;font:500 11px -apple-system,sans-serif;background:rgba(0,0,0,0.62);padding:12px;border-radius:10px;z-index:10;backdrop-filter:blur(10px);display:none;width:210px;';
	document.body.appendChild( panel );

	if ( autoDriver.enabled ) {

		for ( const [ key, min, max, step, label ] of TUNABLES ) {

			const row = document.createElement( 'div' );
			row.style.cssText = 'margin-bottom:9px;';

			const head = document.createElement( 'div' );
			head.style.cssText = 'display:flex;justify-content:space-between;margin-bottom:3px;';
			head.innerHTML = '<span style="opacity:0.7">' + label + '</span><span style="font-variant-numeric:tabular-nums">' + autoDriver[ key ].toFixed( 2 ) + '</span>';

			const slider = document.createElement( 'input' );
			slider.type = 'range';
			slider.min = min; slider.max = max; slider.step = step;
			slider.value = autoDriver[ key ];
			slider.style.cssText = 'width:100%;height:3px;accent-color:#5af168;cursor:pointer;';

			slider.addEventListener( 'input', () => {

				autoDriver[ key ] = parseFloat( slider.value );
				head.lastChild.textContent = parseFloat( slider.value ).toFixed( 2 );

			} );

			row.appendChild( head );
			row.appendChild( slider );
			panel.appendChild( row );

		}

		const hint = document.createElement( 'div' );
		hint.style.cssText = 'opacity:0.45;font-size:10px;margin-top:8px;line-height:1.4;';
		hint.textContent = 'Chỉnh xong copy số vào constructor của AutoDriver.js để lưu.';
		panel.appendChild( hint );

	}

	window.addEventListener( 'keydown', ( e ) => {

		if ( e.code === 'KeyP' && autoDriver.enabled ) {

			autoPilot = ! autoPilot;
			refreshAutoUI();

		}

		if ( e.code === 'KeyO' && autoDriver.debug ) {

			autoDriver.debug.visible = ! autoDriver.debug.visible;

		}

		if ( e.code === 'KeyG' && autoDriver.enabled ) {

			panel.style.display = panel.style.display === 'none' ? 'block' : 'none';

		}

	} );

	const particles = new SmokeTrails( scene );
	const driftMarks = new DriftMarks( scene, mapParam );

	const audio = new GameAudio();
	audio.init( cam.camera, vehicleGroup );

	const lapTimer = new LapTimer( customCells, mapParam );

	const _forward = new THREE.Vector3();
	const _camLead = new THREE.Vector3();

	const contactListener = {
		onContactAdded( bodyA, bodyB ) {

			if ( bodyA !== sphereBody && bodyB !== sphereBody ) return;

			_forward.set( 0, 0, 1 ).applyQuaternion( vehicle.container.quaternion );
			_forward.y = 0;
			_forward.normalize();

			const impactVelocity = Math.abs( vehicle.modelVelocity.dot( _forward ) );
			audio.playImpact( impactVelocity );

		}
	};

	const timer = new THREE.Timer();

	function animate() {

		requestAnimationFrame( animate );

		timer.update();
		const dt = Math.min( timer.getDelta(), 1 / 30 );

		let input = controls.update();

		if ( autoPilot ) {

			// Người chơi chạm vào lái -> tự động tắt autopilot
			if ( Math.abs( input.x ) > 0.05 || Math.abs( input.z ) > 0.05 || input.touchActive ) {

				autoPilot = false;
				refreshAutoUI();

			} else {

				input = autoDriver.update( dt, vehicle );

			}

		}

		updateWorld( world, contactListener, dt );

		vehicle.update( dt, input );

		dirLight.position.set(
			vehicle.spherePos.x + 11.4,
			15,
			vehicle.spherePos.z - 5.3
		);

		const mv = vehicle.modelVelocity;
		_camLead.set( 0, 0, 1 ).applyQuaternion( vehicle.container.quaternion ).multiplyScalar( Math.sqrt( mv.x * mv.x + mv.z * mv.z ) );
		cam.update( dt, vehicle.spherePos, _camLead, vehicle.container.quaternion );
		particles.update( dt, vehicle );
		driftMarks.update( dt, vehicle );
		audio.update( dt, vehicle.linearSpeed / MAX_SPEED, input.z, vehicle.driftIntensity );

		const hasInput = input.touchActive || Math.abs( input.x ) > 0.05 || Math.abs( input.z ) > 0.05;
		lapTimer.update( dt, vehicle.spherePos, hasInput );

		renderer.render( scene, cam.camera );

	}

	animate();

}

init();
