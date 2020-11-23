// copyright 2020 Armin Straller

class robotArm {
    constructor(p, elementWidth, elementHeight, baseLength, armOneLength, armTwoLength, zDiameter, zHeight, drawCoordSys) {
        this._p = p;
        this._divisor = 3;
        this._elementWidth = elementWidth/this._divisor;
        this._elementHeight = elementHeight/this._divisor;
        this._baseLength = baseLength/this._divisor;
        this._armOneLength = armOneLength/this._divisor;
        this._armTwoLength = armTwoLength/this._divisor;
        this._zDia = zDiameter/this._divisor;
        this._zHeight = zHeight/this._divisor;
        this._drawCoord = drawCoordSys;
        this._positionBase = this._p.createVector(0, 0, -300/this._divisor);
        this._armAngles = this._p.createVector(0, 0, 0);
        this._orientation = this._p.createVector(0, 0, 0);
        this._zDistance = 0;
        this._p.createCanvas(p.windowWidth/2, p.windowHeight/2, p.WEBGL);
    }

    draw() {
        this._p.directionalLight(255,255,255, 1,-1,-3);
        this._p.background(255);
        this._p.ambientMaterial(200,200,200);
        this._p.rotateX(this._orientation.x- this._p.PI/4);
        this._p.rotateY(this._orientation.y - this._p.PI/2);
        this._p.rotateZ(this._orientation.z);
        
        this.drawBase();
        this.drawArmOne();
        this.drawArmTwo();
        this.drawZ();

        //draw coordinate System if required
        if(this._drawCoord)
        {
            this.drawBaseCoordinateSystem();
            this.drawCoordinateSystem()
        }
    }

    drawBase(){
        this._p.push();
        this._p.translate(this._positionBase.x, this._positionBase.y - this._elementHeight/2, this._positionBase.z + this._baseLength/2);
        this._p.box(this._elementWidth, this._elementHeight, this._baseLength);
        this._p.pop();
    }

    drawArmOne(){
        let dX = this._p.sin(this._armAngles.x) * this._armOneLength/2;
        let dY = -this._elementHeight - 5/this._divisor;
        let dZ = this._baseLength +  this._p.cos(this._armAngles.x) * this._armOneLength/2;
        this._p.push();
        this._p.translate(this._positionBase.x + dX, this._positionBase.y - this._elementHeight/2 + dY, this._positionBase.z + dZ);
        this._p.rotateY(this._armAngles.x);
        this._p.box(this._elementWidth, this._elementHeight, this._armOneLength);
        this._p.pop();
        
    }
    drawArmTwo(){
        let dX = this._p.sin(this._armAngles.x) * this._armOneLength + this._p.sin(this._armAngles.y + this._armAngles.x) * this._armTwoLength/2;
        let dY = 0;
        let dZ = this._baseLength + (this._p.cos(this._armAngles.x) * this._armOneLength) + this._p.cos(this._armAngles.y + this._armAngles.x) * this._armTwoLength/2;
        this._p.push();
        this._p.translate(this._positionBase.x + dX, this._positionBase.y - this._elementHeight/2 + dY, this._positionBase.z + dZ);
        this._p.rotateY(this._armAngles.y + this._armAngles.x);
        this._p.box(this._elementWidth, this._elementHeight, this._armTwoLength);
        this._p.pop();
    }
    drawZ(){
        let dX = this._p.sin(this._armAngles.x) * this._armOneLength + this._p.sin(this._armAngles.y + this._armAngles.x) * this._armTwoLength;
        let dY = - this._zHeight/2 + this._elementHeight/2 + this._zDistance;
        let dZ = this._baseLength + (this._p.cos(this._armAngles.x) * this._armOneLength) + this._p.cos(this._armAngles.y + this._armAngles.x) * this._armTwoLength;
        this._p.push();
        this._p.translate(this._positionBase.x + dX, this._positionBase.y - this._elementHeight/2 + dY, this._positionBase.z + dZ);
        this._p.rotateY(this._armAngles.z + this._armAngles.y + this._armAngles.x);
        this._p.box(this._zDia, this._zHeight,this._zDia);
        this._p.pop();
    }

    drawBaseCoordinateSystem(){
        // x red, y green z blue
        //x line coord 
        var coordLength = 100/this._divisor;
        this._p.push();
        this._p.stroke(255,0,0);
        this._p.line(this._positionBase.x, this._positionBase.y, this._positionBase.z, this._positionBase.x + coordLength, this._positionBase.y, this._positionBase.z);
        this._p.pop();
        //y line coord 
        this._p.push();
        this._p.stroke(0,255,0);
        this._p.line(this._positionBase.x, this._positionBase.y, this._positionBase.z, this._positionBase.x, this._positionBase.y + coordLength, this._positionBase.z);
        this._p.pop();
        //z line coord 
        this._p.push();
        this._p.stroke(0,0,255);
        this._p.line(this._positionBase.x, this._positionBase.y, this._positionBase.z, this._positionBase.x, this._positionBase.y, this._positionBase.z + coordLength);
        this._p.pop();
    }

    drawCoordinateSystem(){
        let dX = this._p.sin(this._armAngles.x) * this._armOneLength + this._p.sin(this._armAngles.y + this._armAngles.x) * this._armTwoLength;
        let dY = this._zDistance;
        let dZ = this._baseLength + (this._p.cos(this._armAngles.x) * this._armOneLength) + this._p.cos(this._armAngles.y + this._armAngles.x) * this._armTwoLength;
        
        // x red, y green z blue
        //x line coord 
        var coordLength = 100/this._divisor;
        this._p.push();
        this._p.stroke(255,0,0);
        this._p.translate(this._positionBase.x + dX, this._positionBase.y + dY, this._positionBase.z + dZ);
        this._p.rotateY(this._armAngles.z + this._armAngles.y + this._armAngles.x);
        this._p.line(0, 0, 0, 0 + coordLength, 0, 0);
        this._p.pop();
        //y line coord 
        this._p.push();
        this._p.stroke(0,255,0);this._p.translate(this._positionBase.x + dX, this._positionBase.y + dY, this._positionBase.z + dZ);
        this._p.rotateY(this._armAngles.z + this._armAngles.y + this._armAngles.x);
        this._p.line(0, 0, 0, 0, 0 + coordLength, 0);
        this._p.pop();
        //z line coord 
        this._p.push();
        this._p.stroke(0,0,255);
        this._p.translate(this._positionBase.x + dX, this._positionBase.y + dY, this._positionBase.z + dZ);
        this._p.rotateY(this._armAngles.z + this._armAngles.y + this._armAngles.x);
        this._p.line(0, 0, 0, 0, 0, 0 + coordLength);
        this._p.pop();
    }

    set showCoordinateSystem(show){
        this._drawCoord = show;
    }

    set armAngles(a){
        this._armAngles = a;
    }

    set zDistance(z) {
        this._zDistance = z/this._divisor;
    }

    get divisor(){
        return this._divisor;
    }
}