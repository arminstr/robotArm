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
    ranges = [2 * p.PI, 2 * p.PI, 8 * p.PI, 250]
    labels = ['axis 1', 'axis 2', 'axis 3', 'z axis']
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
  var roPos = data.robotPosition;
  p5One.roArm.armAngles = p5One.createVector(roPos.a1, roPos.a2, roPos.a3);
  p5One.roArm.zDistance = roPos.z;
  p5Two.graphOne.addValue([roPos.a1 + p5Two.PI, roPos.a2 + p5Two.PI, roPos.a3 + p5Two.PI * 4, roPos.z]);
}