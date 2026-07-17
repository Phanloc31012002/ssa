import * as THREE from 'three';
import { CELL_RAW, GRID_SCALE, ORIENT_DEG, TRACK_CELLS } from './Track.js';

const S = CELL_RAW * GRID_SCALE;

const _fwd = new THREE.Vector3();
const _to = new THREE.Vector3();
const _vel = new THREE.Vector3();

// Hình học tường (khớp Physics.js): tường tâm ở ±4.75 lưới, dày 0.25
// -> mặt trong ở ±4.5 lưới. Xe là hình cầu bán kính 0.5.
const HALF_ROAD = ( 4.75 - 0.25 ) * GRID_SCALE;   // ~3.38 m
const CAR_RADIUS = 0.5;
const MARGIN = 0.35;
const MAX_OFFSET = HALF_ROAD - CAR_RADIUS - MARGIN;   // ~2.53 m

// Giới hạn quay thật, suy từ Vehicle.update():
//   angularSpeed -> -inputX * grip * 4 * dir,  grip = clamp(|linearSpeed|,0.2,1)
// Ở tốc độ cao grip = 1 => omega_max = 4 rad/s.
const OMEGA_MAX = 4.0;

const PIECE_EXITS = {
	'track-straight': [ 0, 2 ],
	'track-finish':   [ 0, 2 ],
	'track-bump':     [ 0, 2 ],
	'track-corner':   [ 0, 3 ],   // +Z <-> -X ở orient gốc (đã kiểm chứng)
};

const DIR_VEC = [ [ 0, 1 ], [ 1, 0 ], [ 0, -1 ], [ -1, 0 ] ];

function buildCenterline( cells ) {

	// cells == null -> track mặc định. Mảng rỗng -> không có đường đua.
	const list = cells == null ? TRACK_CELLS : cells;
	const map = new Map();

	for ( const [ gx, gz, key, orient ] of list ) {

		const exits = PIECE_EXITS[ key ];
		if ( ! exits ) continue;

		const deg = ORIENT_DEG[ orient ] ?? 0;
		const q = ( ( Math.round( deg / 90 ) % 4 ) + 4 ) % 4;
		map.set( gx + ',' + gz, { gx, gz, key, exits: exits.map( ( d ) => ( d + q ) & 3 ) } );

	}

	if ( map.size === 0 ) return [];

	let start = null;
	for ( const n of map.values() ) if ( n.key === 'track-finish' ) { start = n; break; }
	if ( ! start ) start = map.values().next().value;

	const path = [];
	const seen = new Set();
	let cur = start, cameFrom = -1;

	while ( cur && ! seen.has( cur.gx + ',' + cur.gz ) ) {

		seen.add( cur.gx + ',' + cur.gz );
		path.push( cur );

		let next = null, nf = -1;
		for ( const ex of cur.exits ) {

			if ( ex === cameFrom ) continue;
			const [ dx, dz ] = DIR_VEC[ ex ];
			const cand = map.get( ( cur.gx + dx ) + ',' + ( cur.gz + dz ) );
			if ( cand && cand.exits.includes( ( ex + 2 ) & 3 ) ) { next = cand; nf = ( ex + 2 ) & 3; break; }

		}

		cur = next; cameFrom = nf;

	}

	if ( path.length < 3 ) return [];

	const pts = path.map( ( n ) => new THREE.Vector3( ( n.gx + 0.5 ) * S, 0, ( n.gz + 0.5 ) * S ) );
	const curve = new THREE.CatmullRomCurve3( pts, true, 'catmullrom', 0.5 );
	const n = Math.max( 24, Math.round( curve.getLength() / 1.0 ) );

	const out = [];
	for ( let i = 0; i < n; i ++ ) out.push( curve.getPointAt( i / n ) );
	return out;

}

// Racing line: dịch ngang mỗi điểm trong lòng đường để tối thiểu hoá độ cong.
// Gradient descent trên J = sum |p[i+1] - 2p[i] + p[i-1]|^2
function optimizeRacingLine( center, iterations ) {

	const M = center.length;
	if ( M < 8 ) return { points: center.slice(), offsets: new Float64Array( M ) };

	const nx = new Float64Array( M ), nz = new Float64Array( M );
	for ( let i = 0; i < M; i ++ ) {

		const a = center[ ( i - 1 + M ) % M ], b = center[ ( i + 1 ) % M ];
		const tx = b.x - a.x, tz = b.z - a.z;
		const l = Math.hypot( tx, tz ) || 1;
		nx[ i ] = - tz / l; nz[ i ] = tx / l;

	}

	const off = new Float64Array( M );
	const px = ( i ) => center[ i ].x + nx[ i ] * off[ i ];
	const pz = ( i ) => center[ i ].z + nz[ i ] * off[ i ];

	const lr = 0.004;
	const g = new Float64Array( M );

	for ( let it = 0; it < iterations; it ++ ) {

		g.fill( 0 );

		for ( let i = 0; i < M; i ++ ) {

			const im = ( i - 1 + M ) % M, ip = ( i + 1 ) % M;
			const rx = px( ip ) - 2 * px( i ) + px( im );
			const rz = pz( ip ) - 2 * pz( i ) + pz( im );

			g[ im ] += 2 * ( rx * nx[ im ] + rz * nz[ im ] );
			g[ i ]  += 2 * ( rx * ( - 2 * nx[ i ] ) + rz * ( - 2 * nz[ i ] ) );
			g[ ip ] += 2 * ( rx * nx[ ip ] + rz * nz[ ip ] );

		}

		for ( let i = 0; i < M; i ++ ) {

			off[ i ] = THREE.MathUtils.clamp( off[ i ] - lr * g[ i ], - MAX_OFFSET, MAX_OFFSET );

		}

	}

	const points = [];
	for ( let i = 0; i < M; i ++ ) points.push( new THREE.Vector3( px( i ), 0, pz( i ) ) );
	return { points, offsets: off };

}

// Bán kính cong tại mỗi điểm (qua 3 điểm cách nhau `span` mẫu)
function computeRadii( pts, span ) {

	const M = pts.length;
	const R = new Float64Array( M );

	for ( let i = 0; i < M; i ++ ) {

		const a = pts[ ( i - span + M ) % M ], b = pts[ i ], c = pts[ ( i + span ) % M ];
		const A = Math.hypot( b.x - a.x, b.z - a.z );
		const B = Math.hypot( c.x - b.x, c.z - b.z );
		const C = Math.hypot( c.x - a.x, c.z - a.z );
		const area = Math.abs( ( b.x - a.x ) * ( c.z - a.z ) - ( c.x - a.x ) * ( b.z - a.z ) ) / 2;

		R[ i ] = area > 1e-9 ? ( A * B * C ) / ( 4 * area ) : 1e4;

	}

	// làm mượt để không bị nhiễu răng cưa
	const out = new Float64Array( M );
	for ( let i = 0; i < M; i ++ ) {

		let s = 0;
		for ( let k = -2; k <= 2; k ++ ) s += R[ ( i + k + M ) % M ];
		out[ i ] = s / 5;

	}

	return out;

}

export class AutoDriver {

	constructor( cells, scene ) {

		const center = buildCenterline( cells );
		this.enabled = center.length > 0;
		if ( ! this.enabled ) return;

		const rl = optimizeRacingLine( center, 30000 );
		this.points = rl.points;
		this.center = center;

		// khoảng cách trung bình giữa 2 mẫu (dùng để đổi mét <-> chỉ số)
		let per = 0;
		for ( let i = 0; i < this.points.length; i ++ ) {

			const a = this.points[ i ], b = this.points[ ( i + 1 ) % this.points.length ];
			per += Math.hypot( b.x - a.x, b.z - a.z );

		}
		this.spacing = per / this.points.length;

		this.radii = computeRadii( this.points, 3 );
		this.index = 0;
		this.measuredSpeed = 0;
		this.vMaxSeen = 3;   // tốc độ cao nhất từng đạt (m/s), tự học

		// ── Tinh chỉnh (bấm G trong game để mở bảng) ──
		this.cornerSafety = 0.85;   // <1 = vào cua dè dặt hơn
		this.lookTime = 0.45;       // nhìn trước bao nhiêu giây
		this.lookMin = 2.5;
		this.steerGain = 3.5;
		this.brakeGain = 0;       // phanh gắt cỡ nào khi vượt tốc
		this.driftSlip = 0.10;      // góc trượt (rad) bắt đầu tính là drift
		this.counterSteer = 0.8;   // lái ngược khi trượt đuôi
		this.throttleOnDrift = 0.9;// giữ ga khi drift để đuôi tiếp tục trượt
		// ───────────────────────────────────────────────

		if ( scene ) {

			const mk = ( pts, color, y ) => {

				const g = new THREE.BufferGeometry().setFromPoints(
					pts.concat( [ pts[ 0 ] ] ).map( ( p ) => new THREE.Vector3( p.x, y, p.z ) )
				);
				return new THREE.Line( g, new THREE.LineBasicMaterial( { color } ) );

			};

			this.debug = new THREE.Group();
			this.debug.add( mk( center, 0x3388ff, 0.12 ) );      // xanh dương = tâm đường
			this.debug.add( mk( this.points, 0x00ffaa, 0.18 ) ); // xanh lá  = racing line
			this.debug.visible = false;
			scene.add( this.debug );

		}

	}

	syncIndex( pos ) {

		const n = this.points.length;
		let best = this.index, bestD = Infinity;

		for ( let k = - 8; k <= 50; k ++ ) {

			const i = ( ( this.index + k ) % n + n ) % n;
			const p = this.points[ i ];
			const d = ( p.x - pos.x ) ** 2 + ( p.z - pos.z ) ** 2;
			if ( d < bestD ) { bestD = d; best = i; }

		}

		this.index = best;
		return Math.sqrt( bestD );

	}

	idxAhead( meters ) {

		const n = this.points.length;
		return ( this.index + Math.max( 1, Math.round( meters / this.spacing ) ) ) % n;

	}

	// Tốc độ cho phép tại 1 điểm: v = R * omega_max * an toàn
	speedLimitAt( i ) {

		return this.radii[ i ] * OMEGA_MAX * this.cornerSafety;

	}

	// Tốc độ mục tiêu = min tốc độ cho phép trong quãng phanh phía trước.
	// Nhìn xa theo tốc độ hiện tại -> phanh sớm trước cua gắt.
	targetSpeed( speed ) {

		const n = this.points.length;
		const horizon = Math.max( 6, speed * 1.4 );
		const steps = Math.min( n - 1, Math.max( 2, Math.round( horizon / this.spacing ) ) );

		let v = Infinity;
		for ( let k = 0; k <= steps; k ++ ) {

			const i = ( this.index + k ) % n;
			// càng xa thì càng có thời gian giảm tốc -> nới lỏng dần
			const dist = k * this.spacing;
			const allow = this.speedLimitAt( i ) + dist * 0.55;
			v = Math.min( v, allow );

		}

		return v;

	}

	update( dt, vehicle ) {

		if ( ! this.enabled ) return { x: 0, z: 0, touchActive: false };

		const pos = vehicle.spherePos;
		const dist = this.syncIndex( pos );

		const mv = vehicle.modelVelocity;
		const raw = Math.hypot( mv.x, mv.z );
		this.measuredSpeed = THREE.MathUtils.lerp( this.measuredSpeed, raw, 0.2 );
		const speed = this.measuredSpeed;
		if ( speed > this.vMaxSeen ) this.vMaxSeen = speed;

		_fwd.set( 0, 0, 1 ).applyQuaternion( vehicle.container.quaternion );
		_fwd.y = 0; _fwd.normalize();

		// ── Lái: pure pursuit tới racing line ──
		const look = Math.max( this.lookMin, this.lookMin + speed * this.lookTime );
		const target = this.points[ this.idxAhead( look ) ];

		_to.set( target.x - pos.x, 0, target.z - pos.z );
		if ( _to.lengthSq() < 1e-6 ) return { x: 0, z: 1, touchActive: false };
		_to.normalize();

		const cross = _fwd.x * _to.z - _fwd.z * _to.x;
		const dot = _fwd.dot( _to );
		const angle = Math.atan2( cross, dot );   // + = mục tiêu bên trái

		// inputX âm = rẽ phải (theo Vehicle.update) => dùng +angle
		let steer = angle * this.steerGain;

		// ── Drift: đo góc trượt thật giữa hướng mũi xe và hướng vận tốc ──
		let slip = 0;
		if ( speed > 2.0 ) {

			_vel.set( mv.x, 0, mv.z ).normalize();
			const sc = _fwd.x * _vel.z - _fwd.z * _vel.x;
			const sd = _fwd.dot( _vel );
			slip = Math.atan2( sc, sd );   // + = đuôi trượt sang một bên

		}

		const drifting = Math.abs( slip ) > this.driftSlip;

		// Trượt đuôi -> lái ngược lại để giữ xe không xoay vòng
		if ( drifting ) steer -= slip * this.counterSteer;

		steer = THREE.MathUtils.clamp( steer, - 1, 1 );

		// ── Ga/phanh: bám theo tốc độ mục tiêu suy từ bán kính cua ──
		const vTarget = this.targetSpeed( speed );
		let throttle;

		if ( speed < vTarget ) {

			throttle = 1;   // dưới giới hạn -> full ga

		} else {

			// vượt giới hạn -> giảm ga theo mức vượt
			const over = ( speed - vTarget ) / Math.max( 1, vTarget );
			throttle = THREE.MathUtils.clamp( 1 - over * this.brakeGain, - 1, 1 );

		}

		// Đang drift thì giữ ga để đuôi tiếp tục trượt (đẹp mắt + không mất đà)
		if ( drifting && throttle < this.throttleOnDrift ) throttle = this.throttleOnDrift;

		// Lệch xa racing line -> siết lái, bớt ga
		if ( dist > MAX_OFFSET ) {

			steer = THREE.MathUtils.clamp( steer * 1.6, - 1, 1 );
			throttle = Math.min( throttle, 0.7 );

		}

		// Bị xoay ngược hướng -> ưu tiên lấy lại hướng
		if ( dot < 0 ) {

			throttle = 0.5;
			steer = THREE.MathUtils.clamp( steer * 2, - 1, 1 );

		}

		this.lastSlip = slip;
		this.lastTargetSpeed = vTarget;
		this.drifting = drifting;

		return { x: steer, z: throttle, touchActive: false };

	}

}
