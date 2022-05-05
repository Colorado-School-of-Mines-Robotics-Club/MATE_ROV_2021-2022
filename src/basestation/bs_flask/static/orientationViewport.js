class OrientationViewport{
	constructor(container){
		let parent = this;
		this._setupDone = false;

		this.side_length = 100;

		this.orientation_quaternion = new Quaternion();

		this.gravity = {
			x: 0,
			y: 0,
			z: -9.81
		}
		this.rot = {
			x: 0,
			y: 0,
			z: -50
		}
		this.a = 0;

		this.sketch = new p5(function(p5){
			let frameCount = 0;

			p5.withState = function(block){
				p5.push();
				block();
				p5.pop();
			};

			p5.applyQuaternion = function(quaternion){
				p5.applyMatrix(...quaternion.toMatrix4());
			};

			p5.applyRotation = function(x, y, z, theta){
				p5.applyMatrix(...makeRotation(x, y, z, theta).toMatrix4());
			};

			p5.windowResized = function(){
				let jQuery_containing_element = $(p5.canvas.parentElement);
				p5.resizeCanvas(jQuery_containing_element.innerWidth(), jQuery_containing_element.innerHeight());
			}

			p5.setup = function(){
				p5.createCanvas(0, 0, p5.WEBGL);
				p5.windowResized();
				// parent._onP5Setup();
			}

			p5.draw = function(){
				p5.background(0);

				p5.withState(() => {
					p5.rotateX(-.4);
					p5.rotateY(.5);
					p5.rotateX(p5.PI / 2);
					p5.rotateZ(1);
					p5.applyMatrix(
						1, 0, 0, 0,
						0, -1, 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1
					)
					p5.noFill();

					p5.stroke(255);
					// let v = p5.createVector(parent.rot.x, parent.rot.y, parent.rot.z);
					// v.setMag(50);
					// p5.line(0, 0, 0, v.x, v.y, v.z);
					p5.line(0, 0, 0, parent.rot.x, parent.rot.y, parent.rot.z);
					p5.withState(() => {
						p5.translate(0, 0, -50);
						p5.box(100, 100, 0);
					});
					p5.applyQuaternion(parent.orientation_quaternion);
					// p5.applyRotation(1, 0, 0, parent.a);

					let side_length = parent.side_length;
					let world_axis_length = .5 * parent.side_length;
					let local_axis_length = .6 * parent.side_length;

					p5.stroke(255, 0, 0);
					p5.line(0, 0, 0, world_axis_length, 0, 0);
					p5.stroke(0, 255, 0);
					p5.line(0, 0, 0, 0, world_axis_length, 0);
					p5.stroke(0, 0, 255);
					p5.line(0, 0, 0, 0, 0, world_axis_length);

					p5.withState(() => {
						p5.normalMaterial();
						p5.box(50, 50, 50);
					});


					p5.withState(() => {

						// parent.a += .01;

						// parent.quaternion = {
						// 	w: 0,
						// 	i: p5.cos(parent.a),
						// 	j: 0,
						// 	k: p5.sin(parent.a)
						// };

						// p5.applyMatrix(
						// 	1 - (2 * ((parent.quaternion.j ** 2) + (parent.quaternion.k ** 2))), 2 * ((parent.quaternion.i * parent.quaternion.j) - (parent.quaternion.w * parent.quaternion.k)), 2 * ((parent.quaternion.i * parent.quaternion.k) + (parent.quaternion.w * parent.quaternion.j)), 0,
						// 	2 * ((parent.quaternion.i * parent.quaternion.j) + (parent.quaternion.w * parent.quaternion.k)), 1 - (2 * ((parent.quaternion.i ** 2) + (parent.quaternion.k ** 2))), 2 * ((parent.quaternion.j * parent.quaternion.k) - (parent.quaternion.w * parent.quaternion.i)), 0,
						// 	2 * ((parent.quaternion.i * parent.quaternion.k) - (parent.quaternion.w * parent.quaternion.j)), 2 * ((parent.quaternion.j * parent.quaternion.k) + (parent.quaternion.w * parent.quaternion.i)), 1 - (2 * ((parent.quaternion.i ** 2) + (parent.quaternion.j ** 2))), 0,
						// 	0, 0, 0, 1
						// );
						//
						// p5.stroke(255);
						// p5.box(side_length, side_length, side_length);
						//
						// p5.stroke(255, 0, 0);
						// p5.line(-local_axis_length, -local_axis_length, -local_axis_length, local_axis_length, -local_axis_length, -local_axis_length);
						// p5.stroke(0, 255, 0);
						// p5.line(-local_axis_length, -local_axis_length, -local_axis_length, -local_axis_length, local_axis_length, -local_axis_length);
						// p5.stroke(0, 0, 255);
						// p5.line(-local_axis_length, -local_axis_length, -local_axis_length, -local_axis_length, -local_axis_length, local_axis_length);

					});
				});

				p5.withState(() => {
					p5.ortho();
					p5.translate(100, 100, 0);
					p5.strokeWeight(3);
					p5.withState(() => {
						p5.noStroke();
						p5.rotate(p5.atan2(parent.rot.x, parent.rot.z));
						p5.fill(40, 140, 200);
						p5.arc(0, 0, 100, 100, 0, p5.PI, p5.OPEN);
						p5.fill(200, 140, 40);
						p5.arc(0, 0, 100, 100, p5.PI, p5.TWO_PI, p5.OPEN);
						p5.withState(() => {
							let pitch = -p5.atan2(parent.rot.z, parent.rot.y);
							let mid_color;
							p5.rotateX(pitch);
							if(Math.abs(pitch) <= p5.HALF_PI){
								mid_color = p5.color(200, 140, 40);
							}else{
								mid_color = p5.color(40, 140, 200);
							}
							p5.stroke(mid_color);
							p5.line(-50, 0, 50, 0);
							p5.stroke(191);
							p5.fill(mid_color);
							p5.ellipse(0, 0, 100, 100);

							p5.stroke(63);
							p5.noFill();
							p5.strokeWeight(2);
							for(let angle = -90; angle <= 90; angle += 15){
								if(angle == 0) continue;
								p5.withState(() => {
									p5.rotateX(p5.radians(angle));
									if(angle % 30 == 0){
										p5.strokeWeight(2);
									}else{
										p5.strokeWeight(1);
									}
									p5.arc(0, 0, 100, 100, p5.HALF_PI - .25, p5.HALF_PI + .25);
								});
							}
						});
						p5.stroke(191);
						p5.noFill();
						p5.withState(() => {
							p5.translate(0, 0, 50);
							p5.ellipse(0, 0, 100, 100);
						});
					});

					p5.stroke(200, 100, 0);
					p5.strokeWeight(2);
					p5.line(-30, 0, 50, -10, 0, 50);
					p5.line(10, 0, 50, 30, 0, 50);
				});
			}
		}, container);
	}

	_onP5Setup(){
		this.canvas = this.sketch.canvas;
		this.container_element = this.canvas.parentElement;

		this._setupDone = true;
	}
}
