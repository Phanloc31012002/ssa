import * as THREE from 'three';

const _desired = new THREE.Vector3();
const _lookPoint = new THREE.Vector3();
const _carForward = new THREE.Vector3();
const _flat = new THREE.Vector3();

export class Camera {

	constructor() {

		this.camera = new THREE.PerspectiveCamera( 55, window.innerWidth / window.innerHeight, 0.1, 200 );

		// Third-person chase settings
		this.distance = 7.5;        // khoảng cách phía sau xe
		this.height = 3.0;          // độ cao camera
		this.lookHeight = 1.2;      // điểm ngắm cao hơn xe một chút
		this.rotationSmoothing = 4.0;  // độ trễ khi xe quay (thấp = mượt/lười hơn)
		this.positionSmoothing = 10.0; // độ bám vị trí

		this.smoothedForward = new THREE.Vector3( 0, 0, 1 );
		this.smoothedPos = new THREE.Vector3();
		this.smoothedLook = new THREE.Vector3();
		this.initialized = false;

		// debug object giữ lại để main.js không lỗi
		this.debug = new THREE.Object3D();
		this.debug.visible = false;

		window.addEventListener( 'resize', () => {

			this.camera.aspect = window.innerWidth / window.innerHeight;
			this.camera.updateProjectionMatrix();

		} );

	}

	// target: vị trí xe, velocity: vector hướng chạy (đã nhân tốc độ), quaternion: hướng thân xe
	update( dt, target, velocity, quaternion ) {

		if ( quaternion ) {

			_carForward.set( 0, 0, 1 ).applyQuaternion( quaternion );

		} else {

			_carForward.copy( velocity );

		}

		_carForward.y = 0;
		if ( _carForward.lengthSq() < 1e-6 ) _carForward.copy( this.smoothedForward );
		_carForward.normalize();

		const aRot = this.initialized ? 1 - Math.exp( - dt * this.rotationSmoothing ) : 1;
		this.smoothedForward.lerp( _carForward, aRot );
		this.smoothedForward.y = 0;
		this.smoothedForward.normalize();

		// vị trí mong muốn: lùi về sau xe theo hướng đã làm mượt
		_desired.copy( target )
			.addScaledVector( this.smoothedForward, - this.distance );
		_desired.y = target.y + this.height;

		_lookPoint.copy( target );
		_lookPoint.y += this.lookHeight;

		const aPos = this.initialized ? 1 - Math.exp( - dt * this.positionSmoothing ) : 1;
		this.smoothedPos.lerp( _desired, aPos );
		this.smoothedLook.lerp( _lookPoint, aPos );
		this.initialized = true;

		this.camera.position.copy( this.smoothedPos );
		this.camera.lookAt( this.smoothedLook );

	}

}
