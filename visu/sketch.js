// copyright 2020 Armin Straller
var sketchOne = function(p) {
  p.roArm;
  p.setup = function() {
    p.roArm = new robotArm(p,90,55,165,250,250,20,275, true);
  }
  p.draw = function() {
    update();
    p.roArm.draw();
  }
}
var p5One = new p5(sketchOne);

var sketchTwo = function(p) {
  p.graphOne;
  p.setup = function() {
    ranges = [2 * p.PI, 2 * p.PI, 250, 8 * p.PI]
    labels = ['axis 1', 'axis 2', 'z axis', 'axis 3',]
    p.graphOne = new graph(p, p.windowWidth/2, ranges, labels);
  }
  p.draw = function() {
    p.graphOne.draw();
  }
}

var p5Two = new p5(sketchTwo);

function update() {
  p5One.loadJSON("../control.json", gotData);
}

function gotData(data) {
  var axisPosition = data.axisPos;
  p5One.roArm.armAngles = p5One.createVector(axisPosition.a1, axisPosition.a2, axisPosition.a3);
  p5One.roArm.zDistance = axisPosition.z;
  p5Two.graphOne.addValue([axisPosition.a1 + p5Two.PI, axisPosition.a2 + p5Two.PI, axisPosition.z, axisPosition.a3 + p5Two.PI * 4]);
}