class graph {
    constructor(p, width, ranges, labels){
        this._p = p;
        this._margin = 12;
        this._readPos = 0;
        this._writePos = 0;
        this._valuesX = [];
        this._height = 65;
        this._numPts = 500;
        this._ranges = ranges;
        this._labels = labels;
        this._textSize = 10;
        this._numRanges = ranges.length;
        this._p.createCanvas(width, this._height * this._numRanges);
    }

    draw(){
        this._p.background(255);
        this.drawLines();
        this.drawLabels();
    }

    drawLines(){
        if(this._valuesX.length > 0) {
            this._p.stroke(0);
            // draw lines
            let px = [];
            for(let k = 0; k < this._numRanges; k++){
                px.push(0);
            }

            let py = this._valuesX[0];
            
            for(let i = 0; i < this._valuesX.length; i++){
                let x = [];
                let y = [];
                for(let j = 0; j < this._numRanges; j++){
                    x.push(i * (this._p.width / (this._numPts-1)));
                }
                y = this._valuesX[i];
                for(let j = 0; j < this._numRanges; j++){
                    this._p.line(px[j], py[j], x[j], y[j]);
                }
                //store the last position
                px = x;
                py = y;
            } 
        }
    }

    drawLabels(){
        this._p.textSize(this._textSize);
        this._p.noStroke();
        for(let j = 0; j < this._numRanges; j++){
            this._p.text(this._labels[j], 5, this._textSize + j * this._height);
        }
        
    }

    addValue(val){
        if(val.length > this._numValues - 1 ){
            console.log("Error: More values then graph expected.")
        }
        var values = [];
        for(let j = 0; j < this._numRanges; j++){
            values.push((val[j] / this._ranges[j]) * (this._height - this._margin * 2) + this._margin + j * this._height);
        }
        this._valuesX.unshift(values);
        if(this._valuesX.length > this._numPts){
            this._valuesX.pop();
        }
    }

}