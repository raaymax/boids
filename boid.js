

const GROUP_RADIUS = 5.0;
const AVOID_RADIUS = 3.0;
const UPDATE_RATE = 0.05;
const SPEED_LIMIT = 10.0;
const BOUNDARY = 50.0;
const AVOID_CURSOR_RADIUS = 10.0;

/*const Vec = BABYLON.Vector3;

Vec.rand = function(scale){
    let v = new Vec();
    v.x = Math.random()*2-1;
    v.y = Math.random()*2-1;
    v.z = Math.random()*2-1;
    return v.scaleInPlace(scale);
};
*/


class Vec{
    constructor(x=0,y=0,z=0){
        this.data = [x,y,z];
    }

    static Zero(){
        return new Vec();
    }

    static Distance(v1,v2){
        return v2.clone().sub(v1).len();
    }

    static Rand(scale){
        return (new Vec()).exec(()=>(Math.random()*2-1)*scale);
    }

    static Clone({x,y,z}){
        return new Vec(x,y,z);
    }

    get x(){ return this.data[0]; }
    get y(){ return this.data[1]; }
    get z(){ return this.data[2]; }

    set x(v){ this.data[0] = v; }
    set y(v){ this.data[1] = v; }
    set z(v){ this.data[2] = v; }

    clone(){
        return new Vec(this.x, this.y, this.z);
    }

    exec(fn){
        this.data.forEach((v,k,a)=>{
            a[k] = fn(v, k);
        });
        return this;
    }

    mulp(v){
        return this.exec((c,k)=>c*v);
    }

    mul(v){
        return this.exec((c,k)=>c*v.data[k]);
    }

    divp(v){
        return this.exec((c,k)=>c/v);
    }

    div(v){
        return this.exec((c,k)=>c/v.data[k]);
    }
    subp(v){
        return this.exec((c,k)=>c-v);
    }
    sub(v){
        return this.exec((c,k)=>c-v.data[k]);
    }

    addp(v){
        return this.exec((c,k)=>c+v);
    }
    add(v){
        return this.exec((c,k)=>c+v.data[k]);
    }
    len(){
        return Math.sqrt(
            Math.pow(this.x, 2)+
            Math.pow(this.y, 2)+
            Math.pow(this.z, 2));
    }

    toString(){
        return "[Vec "+this.x+" "+this.y+" "+this.z+"] ";
    }
}




class Swarm{
    constructor(n){
        this.nextBoidId = 0;
        this.boids = Array(n).fill(null).map(()=>new Boid(this));
        //this.updateContainer();
        //console.log(this.container.getNearby(this.boids[0].pos));

    }

    updateNeighbors(){
        for(let j = 0; j < this.boids.length; j++){
            for(let i = j+1; i < this.boids.length; i++){
                let boid = this.boids[j];
                let other = this.boids[i];
                let x = Math.abs(boid.pos.x - other.pos.x);
                let y = Math.abs(boid.pos.y - other.pos.y);
                let z = Math.abs(boid.pos.z - other.pos.z);
                if(x <= AVOID_RADIUS && y <= AVOID_RADIUS && z <= AVOID_RADIUS){
                    boid.neighbors.push(other);
                    other.neighbors.push(boid);
                    boid.avoiders.push(other);
                    other.avoiders.push(boid);
                }else if((x <= GROUP_RADIUS) && (y <= GROUP_RADIUS) && (z <= GROUP_RADIUS)){
                    boid.neighbors.push(other);
                    other.neighbors.push(boid);
                }
            }
        }

    }
    find(pos, radius, id, boids = this.boids){
        //return this.container.getNearby(pos).filter(boid=>{
        return boids.filter(boid=>{
            return boid.id !== id &&
                (Math.abs(boid.pos.x - pos.x) <= radius ) &&
                (Math.abs(boid.pos.y - pos.y) <= radius ) &&
                (Math.abs(boid.pos.z - pos.z) <= radius )// &&
                //Vec.Distance(boid.pos, pos) <= radius;
        });
    }
    update() {
        /*this.boids.forEach(boid => {
            boid.neighbors = [];
            boid.avoiders=[];
        });*/
        //this.updateNeighbors();
        this.boids.forEach(boid => boid.update());
        //this.boids.forEach(boid => boid.move());
    }
    cursor(a,b){
        this.line = {a,b};
    }
}


class Boid{
    constructor(env){
        this.neighbors = [];
        this.avoiders = [];
        this.env = env;
        this.id = this.env.nextBoidId++;
        this.velocity = Vec.Rand(10.0);
        this.pos = Vec.Rand(BOUNDARY);
    }

    center(arr){
        if(arr.length ==0) return Vec.Zero();
        return arr.reduce((ret, boid)=>{
            return ret.add(boid.pos);
        }, Vec.Zero())
            .divp(arr.length)
            .sub(this.pos)
            .divp(10.0)
    }

    heading(arr){
        if(arr.length == 0) return Vec.Zero();
        return arr.reduce((ret, boid)=>{
            return ret.add(boid.velocity);
        }, Vec.Zero())
            .divp(arr.length)
            .sub(this.velocity)
            .divp(2.0)
    }

    avoid(arr){
        return arr.reduce((ret, boid)=>{
            return ret.sub(boid.pos).add(this.pos);
        }, Vec.Zero())
            .divp(2.0)
    }

    limit_velocity(){
        if(this.velocity.len() > SPEED_LIMIT){
            this.velocity.divp(this.velocity.len()).mulp(SPEED_LIMIT);
        }
    }

    avoid_walls(){
        if(this.pos.len() > BOUNDARY){
            return this.pos.clone().mulp(-0.1);
        }
        return Vec.Zero();
    }

    avoid_cursor(){
        let p = this.pos;
        if(!this.env.line) return Vec.Zero();
        let a = this.env.line.a;
        let b = this.env.line.b;

        let ab = b.subtract(a);
        let ap = p.subtract(a);

        let area = Vec.Cross(ap,ab).lengthSquared();
        let s = ab.lengthSquared();
        let d = area / s;

        let t = Math.sqrt(ap.length() + Math.pow(d,2));
        let P = a.add(ab.clone().normalize().scaleInPlace(-t));


        let v = p.subtract(P);

        if(d < AVOID_CURSOR_RADIUS){
            return v.scaleInPlace(10.0);
        }
        return Vec.Zero();
    }

    update(){

        let neighbors = this.env.find(this.pos, GROUP_RADIUS, this.id);
        let avoiders = this.env.find(this.pos, AVOID_RADIUS, this.id, neighbors);

//        if(check(neighbors, this.neighbors))


        let center = this.center(neighbors);
        let heading = this.heading(neighbors);
        let avoid = this.avoid(avoiders);
        let avoidWalls = this.avoid_walls().mulp(0.1);

        this.velocity
            .add(center)
            .add(heading)
            .add(avoid)
            .add(avoidWalls);

        this.limit_velocity();
        this.pos.add(this.velocity.clone().mulp(UPDATE_RATE));
    }

    move(){
    }
    /*
    check(arr1,arr2){
        let ret = false;
        arr.forEach(b1=>{
            if(!arr2.find(b=>b1.id == b.id)){
                ret = true;
            }
        })
        return ret;
    }*/

}

