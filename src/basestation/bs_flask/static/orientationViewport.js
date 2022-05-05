class OrientationViewport{
	constructor(container){
		let parent = this;
		this._setupDone = false;

		this.side_length = 100;

		this.quaternion = {
			w: 0,
			i: 0,
			j: 0,
			k: 0
		};

		this.a = 0;

		this.sketch = new p5(function(p5){
			let frameCount = 0;

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
				// p5.rotateX(-.4);
				// p5.rotateY(.5);
				p5.applyMatrix(
					1, 0, 0, 0,
					0, -1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1
				)
				p5.noFill();


				let side_length = parent.side_length;
				let world_axis_length = .5 * parent.side_length;
				let local_axis_length = .6 * parent.side_length;

				p5.stroke(255, 0, 0);
				p5.line(0, 0, 0, world_axis_length, 0, 0);
				p5.stroke(0, 255, 0);
				p5.line(0, 0, 0, 0, world_axis_length, 0);
				p5.stroke(0, 0, 255);
				p5.line(0, 0, 0, 0, 0, world_axis_length);

				p5.push();

				parent.a += .01;

				// parent.quaternion = {
				// 	w: 0,
				// 	i: p5.cos(parent.a),
				// 	j: 0,
				// 	k: p5.sin(parent.a)
				// };

				p5.applyMatrix(
					1 - (2 * ((parent.quaternion.j ** 2) + (parent.quaternion.k ** 2))), 2 * ((parent.quaternion.i * parent.quaternion.j) - (parent.quaternion.w * parent.quaternion.k)), 2 * ((parent.quaternion.i * parent.quaternion.k) + (parent.quaternion.w * parent.quaternion.j)), 0,
					2 * ((parent.quaternion.i * parent.quaternion.j) + (parent.quaternion.w * parent.quaternion.k)), 1 - (2 * ((parent.quaternion.i ** 2) + (parent.quaternion.k ** 2))), 2 * ((parent.quaternion.j * parent.quaternion.k) - (parent.quaternion.w * parent.quaternion.i)), 0,
					2 * ((parent.quaternion.i * parent.quaternion.k) - (parent.quaternion.w * parent.quaternion.j)), 2 * ((parent.quaternion.j * parent.quaternion.k) + (parent.quaternion.w * parent.quaternion.i)), 1 - (2 * ((parent.quaternion.i ** 2) + (parent.quaternion.j ** 2))), 0,
					0, 0, 0, 1
				);

				p5.stroke(255);
				p5.box(side_length, side_length, side_length);

				p5.stroke(255, 0, 0);
				p5.line(-local_axis_length, -local_axis_length, -local_axis_length, local_axis_length, -local_axis_length, -local_axis_length);
				p5.stroke(0, 255, 0);
				p5.line(-local_axis_length, -local_axis_length, -local_axis_length, -local_axis_length, local_axis_length, -local_axis_length);
				p5.stroke(0, 0, 255);
				p5.line(-local_axis_length, -local_axis_length, -local_axis_length, -local_axis_length, -local_axis_length, local_axis_length);

				p5.pop();
				frameCount++;
			}
		}, container);
	}

	_onP5Setup(){
		this.canvas = this.sketch.canvas;
		this.container_element = this.canvas.parentElement;

		this._setupDone = true;
	}
}
