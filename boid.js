

const GROUP_RADIUS = 50.0;
const AVOID_RADIUS = 5.0;
const UPDATE_RATE = 0.08;
const SPEED_LIMIT = 10.0;
const BOUNDARY = 50.0;
const AVOID_CURSOR_RADIUS = 100.0;
const Vec = BABYLON.Vector3;

Vec.rand = function(scale){
    let v = new Vec();
    v.x = Math.random()*2-1;
    v.y = Math.random()*2-1;
    v.z = Math.random()*2-1;
    return v.scaleInPlace(scale);
};


class Environment{
    constructor(n){
        this.nextBoidId = 0;
        this.boids = Array(n).fill(null).map(()=>new Boid(this));
    }
    find(pos, radius, id){
        return this.boids.filter(boid=>{
            return boid.id !== id && Vec.DistanceSquared(boid.pos, pos) <= radius;
        })
    }
    update(){
        this.boids.forEach(boid=>boid.update());
    }
    cursor(a,b){
        this.line = {a,b};
    }
}


class Boid{
    constructor(env){
        this.env = env;
        this.id = this.env.nextBoidId++;
        this.velocity = Vec.Zero();//rand(10.0);
        this.pos = Vec.rand(20.0);
    }

    center(){
        let arr = this.env.find(this.pos, GROUP_RADIUS, this.id);
        if(!arr) return Vec.Zero();
        return arr.reduce((ret, boid)=>{
            return ret.addInPlace(boid.pos);
        }, Vec.Zero())
            .scaleInPlace(1.0/arr.length)
            .subtractInPlace(this.pos)
            .scaleInPlace(0.1)
    }

    heading(){
        let arr = this.env.find(this.pos, GROUP_RADIUS, this.id);
        if(!arr) return Vec.Zero();
        return arr.reduce((ret, boid)=>{
            return ret.addInPlace(boid.velocity);
        }, Vec.Zero())
            .scaleInPlace(1.0/arr.length)
            .subtractInPlace(this.velocity)
            .scaleInPlace(0.2)
    }

    avoid(){
        let arr = this.env.find(this.pos, AVOID_RADIUS, this.id);
        return arr.reduce((ret, boid)=>{
            return ret.subtractInPlace(boid.pos.subtract(this.pos));
        }, Vec.Zero())
            .scaleInPlace(0.5)
    }

    limit_velocity(){
        if(this.velocity.lengthSquared() > SPEED_LIMIT){
            this.velocity.scaleInPlace(SPEED_LIMIT/this.velocity.lengthSquared());
        }
    }

    avoid_walls(){
        let v = Vec.Zero();
        let pos = this.pos;
        ['x','y','z'].forEach(k=>{
            if(pos[k] > BOUNDARY){
                v[k] = -( pos[k] + BOUNDARY );
            }else if(pos[k] < -BOUNDARY){
                v[k] = -( pos[k] - BOUNDARY );
            }
        });
        return v;
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
        let center = this.center();
        let heading = this.heading();
        let avoid = this.avoid();
        let avoidWalls = this.avoid_walls().scale(0.1);
        let avoidCursor = this.avoid_cursor();
        this.velocity
            .addInPlace(center)
            //.addInPlace(heading)
            //.addInPlace(avoid)
            .addInPlace(avoidWalls)
            .addInPlace(avoidCursor);

        this.limit_velocity();

        this.pos.addInPlace(this.velocity.scale(UPDATE_RATE));
    }

}

