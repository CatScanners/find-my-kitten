
// Some attemps at improving the code

// Location gradient decent combined with rotation force sphere.
// 
DroneState gradientDescentLocateDrone(const std::vector<vector3D>& positions, const std::vector<vector2D>& features, DroneState previousState) {
    if (positions.size() != features.size()) std::cout << "positions and their coresponding positions on camera do not match\n";
    vector3D gradientLoc = EMPTY_VECTOR3D;
            Sphere rotationSphere = {positions.size(),EMPTY_VECTOR3D};
    float fitness = 0.0f;
    for (int i = 0; i < positions.size(); i++){
        auto& [x,y,z] = positions[i];
        auto& [w,h] = features[i];
        auto& [locx,locy,locz] = previousState.loc; 
        auto [x1,y1,z1] = previousState.forwardRot();
        auto [x2,y2,z2] = previousState.rightRot();
        auto [x3,y3,z3] = previousState.downRot();
        float fw = ((y2*z3-y3*z2)*(x-locx)-(x2*z3-x3*z2)*(y-locy)+(x2*y3-x3*y2)*(z-locz))*w+((y1*z3-y3*z1)*(x-locx)-(x1*z3-x3*z1)*(y-locy)+(x1*y3-x3*y1)*(z-locz)); //= 0
        float dxfw = -(y2*z3-y3*z2)*w-y1*z3+y3*z1;
        float dyfw =  (x2*z3-x3*z2)*w+x1*z3-x3*z1;
        float dzfw = -(x2*y3-x3*y2)*w-x1*y3+x3*y1;
        vector3D gradientLocw = { 2*fw*dxfw, 2*fw*dyfw, 2*fw*dzfw };
        float fh = ((y2*z3-y3*z2)*(x-locx)-(x2*z3-x3*z2)*(y-locy)+(x2*y3-x3*y2)*(z-locz))*h-((y1*z2-y2*z1)*(x-locx)-(x1*z2-x2*z1)*(y-locy)+(x1*y2-x2*y1)*(z-locz)); //= 0
        float dxfh = -h*(y2*z3-y3*z2)+y1*z2-y2*z1;
        float dyfh =  h*(x2*z3-x3*z2)-x1*z2+x2*z1;
        float dzfh = -h*(x2*y3-x3*y2)+x1*y2-x2*y1;
        vector3D gradientLoch = { 2*fh*dxfh, 2*fh*dyfh, 2*fh*dzfh };
        gradientLoc -= gradientLocw;
        gradientLoc -= gradientLoch;
        fitness += fw*fw+fh*fh;

        vector3D vec3D = (positions[i]-previousState.loc).normalize()                                                 ;
        vector3D vec2D = (previousState.forwardRot()+previousState.rightRot()*w+previousState.downRot()*h).normalize();
        vector3D forceOnSphere = vec3D.projectToNormal(vec2D);
        rotationSphere.applyImpulse(forceOnSphere,vec2D);
    }
    DroneState newState = previousState;
    newState.loc = previousState.loc + gradientLoc/positions.size()*0.5;
    newState.rot = rotationSphere.toQuaternion(0.2f)*newState.rot;
    return newState;
}

// reliable?
DroneState gradientDescentLocateDrone(const std::vector<vector3D>& positions, const std::vector<vector2D>& features, const DroneState previousState, const bool lockZ , const bool display) {
    if (positions.size() != features.size()) std::cout << "positions and their coresponding positions on camera do not match\n";
    DroneState bestState = previousState;
    float Minfitness = 340282346638528859811704183484516925440.0000000000000000f; 
    DroneState newState = previousState;
    constexpr int maxLoops = 100000;
    
    for (int loopNum = 0; loopNum < maxLoops; loopNum++){ //&& smallChanges < maxAmountOfSmallChanges
        vector3D gradientLoc = EMPTY_VECTOR3D;
                Sphere rotationSphere = {positions.size(),EMPTY_VECTOR3D};
        float fitness = 0.0f;
        for (int i = 0; i < positions.size(); i++){
            auto& [x,y,z] = positions[i];
            auto& [w,h] = features[i];
            //// optimal location
            //auto& [locx,locy,locz] = newState.loc; 
            //auto [x1,y1,z1] = newState.forwardRot();
            //auto [x2,y2,z2] = newState.rightRot();
            //auto [x3,y3,z3] = newState.downRot();
            //float fw = ((y2*z3-y3*z2)*(x-locx)-(x2*z3-x3*z2)*(y-locy)+(x2*y3-x3*y2)*(z-locz))*w+((y1*z3-y3*z1)*(x-locx)-(x1*z3-x3*z1)*(y-locy)+(x1*y3-x3*y1)*(z-locz)); //= 0
            //float dxfw = -(y2*z3-y3*z2)*w-y1*z3+y3*z1;
            //float dyfw =  (x2*z3-x3*z2)*w+x1*z3-x3*z1;
            //float dzfw = -(x2*y3-x3*y2)*w-x1*y3+x3*y1;
            //vector3D gradientLocw = { 2*fw*dxfw, 2*fw*dyfw, 2*fw*dzfw };
            //float fh = ((y2*z3-y3*z2)*(x-locx)-(x2*z3-x3*z2)*(y-locy)+(x2*y3-x3*y2)*(z-locz))*h-((y1*z2-y2*z1)*(x-locx)-(x1*z2-x2*z1)*(y-locy)+(x1*y2-x2*y1)*(z-locz)); //= 0
            //float dxfh = -h*(y2*z3-y3*z2)+y1*z2-y2*z1;
            //float dyfh =  h*(x2*z3-x3*z2)-x1*z2+x2*z1;
            //float dzfh = -h*(x2*y3-x3*y2)+x1*y2-x2*y1;
            //vector3D gradientLoch = { 2*fh*dxfh, 2*fh*dyfh, 2*fh*dzfh };
            //gradientLoc -= gradientLocw;
            //gradientLoc -= gradientLoch;
            //fitness += fw*fw+fh*fh;

            // optimal rotation
            vector3D vec3D = (positions[i]-newState.loc).normalize();
            vector3D vec2D = (newState.forwardRot()+newState.rightRot()*w+newState.downRot()*h).normalize();
            vector3D forceOnSphere = vec3D.projectToNormal(vec2D);
            float sing = (newState.forwardRot().dot(forceOnSphere) >= 0)*2-1;
            vector3D movement = ((vec3D-vec2D)-forceOnSphere)*sing;
            gradientLoc -= movement;
            rotationSphere.applyImpulse(forceOnSphere,vec2D);

        }
        //if (fitness < Minfitness){
        //    Minfitness = fitness;
        //    bestState = newState;
        //}
        if (lockZ){
            gradientLoc.z = 0;
            newState.loc = newState.loc + gradientLoc/positions.size()*0.5f;
            newState.rot = rotationSphere.toQuaternion(0.07f).z_axis_component()*newState.rot;
        }else {
            newState.loc = newState.loc + gradientLoc/positions.size()*0.5f;
            newState.rot = rotationSphere.toQuaternion(0.07f)*newState.rot;
        }
        if (display){
            drone temp = {newState};
            temp.display(positions,features);
            //if (count == extraIterationsV2) {
            //    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            //}

        }
        //std::cout << "fitness: " << fitness << "\n";
    }
    return newState;
}

// IF given correct orientation then This will produce correct location VERY FAST.
DroneState optimalLocation(const std::vector<vector3D>& positions, const std::vector<vector2D>& features, DroneState previousState) {
    if (positions.size() != features.size()) std::cout << "positions and their coresponding positions on camera do not match\n";
    vector3D gradientLoc = EMPTY_VECTOR3D;
    DroneState newState = previousState;
    
    for (int n = 0; n < 100; n++){
        float fitness = 0.0f;
        for (int i = 0; i < positions.size(); i++){
            auto& [x,y,z] = positions[i];
            auto& [w,h] = features[i];
            auto& [locx,locy,locz] = newState.loc; 
            auto [x1,y1,z1] = newState.forwardRot();
            auto [x2,y2,z2] = newState.rightRot();
            auto [x3,y3,z3] = newState.downRot();
            float fw = ((y2*z3-y3*z2)*(x-locx)-(x2*z3-x3*z2)*(y-locy)+(x2*y3-x3*y2)*(z-locz))*w+((y1*z3-y3*z1)*(x-locx)-(x1*z3-x3*z1)*(y-locy)+(x1*y3-x3*y1)*(z-locz)); //= 0
            float dxfw = -(y2*z3-y3*z2)*w-y1*z3+y3*z1;
            float dyfw =  (x2*z3-x3*z2)*w+x1*z3-x3*z1;
            float dzfw = -(x2*y3-x3*y2)*w-x1*y3+x3*y1;
            vector3D gradientLocw = { 2*fw*dxfw, 2*fw*dyfw, 2*fw*dzfw };
            float fh = ((y2*z3-y3*z2)*(x-locx)-(x2*z3-x3*z2)*(y-locy)+(x2*y3-x3*y2)*(z-locz))*h-((y1*z2-y2*z1)*(x-locx)-(x1*z2-x2*z1)*(y-locy)+(x1*y2-x2*y1)*(z-locz)); //= 0
            float dxfh = -h*(y2*z3-y3*z2)+y1*z2-y2*z1;
            float dyfh =  h*(x2*z3-x3*z2)-x1*z2+x2*z1;
            float dzfh = -h*(x2*y3-x3*y2)+x1*y2-x2*y1;
            vector3D gradientLoch = { 2*fh*dxfh, 2*fh*dyfh, 2*fh*dzfh };
            gradientLoc -= gradientLocw;
            gradientLoc -= gradientLoch;
            fitness += fw*fw+fh*fh;
        }
        newState.loc = newState.loc + gradientLoc/positions.size()*0.5;
    }
    return newState;
}


void gradientStep(const std::vector<vector3D>& positions, const std::vector<vector2D>& features, DroneState &previousState) {
    previousState = optimalRotation(positions, features, previousState);
    float quant = pointSD(positions)*0.001;
    float fit  = fitness(positions, features, previousState);
    previousState.loc.x += quant;
    float fitx = (fitness(positions, features, previousState)-fit);
    previousState.loc.x -= quant;
    previousState.loc.y += quant;
    float fity = (fitness(positions, features, previousState)-fit);
    previousState.loc.y -= quant;
    previousState.loc.z += quant;
    float fitz = (fitness(positions, features, previousState)-fit);
    previousState.loc.z += quant;
    vector3D gradient = {fitx,fity,fitz};
    previousState.loc -= gradient;
}

struct instance {
    DroneState state;
    std::vector<vector2D> render;
    vector3D dir(int p){
        return state.forwardRot()+state.rightRot()*render[p].x+state.downRot()*render[p].y;
    }
    vector3D loc(){
        return state.loc;
    }
};

// trying to correct 3D point positions.

void pointGradientDecent(std::vector<vector3D>& positions, std::vector<instance>& frames) {

    for (int p = 0; p < positions.size(); p++){   
        vector3D dv = EMPTY_VECTOR3D;
        for (int i = 0; i < frames.size(); i++){
            auto&[x, y, z ]   = positions[p];
            auto [vx,vy,vz] = frames[i].dir(p);
            auto [px,py,pz] = frames[i].loc();
            //float dx = (((vy*vy+vz*vz)*x-vx*vy*y-vx*vz*z-px*(vy*vy+vz*vz)+(py*vy+pz*vz)*vx) /(std::sqrt((vy*vy+vz*vz)*x*x-2*x*(vx*vy*y+vx*vz*z+px*(vy*vy+vz*vz)-(py*vy+pz*vz)*vx)+(vx*vx+vz*vz)*y*y-2*y*(vy*vz*z-px*vx*vy+py*(vx*vx+vz*vz)-pz*vy*vz)+(vx*vx+vy*vy)*z*z+2*(px*vx*vz+py*vy*vz-pz*(vx*vx+vy*vy))*z+px*px*(vy*vy+vz*vz)-2*px*(py*vy+pz*vz)*vx+py*py*(vx*vx+vz*vz)-2*py*pz*vy*vz+pz*pz*(vx*vx+vy*vy))));
            //float dy = (((vx*vx+vz*vz)*y-vx*vy*x-vy*vz*z+px*vx*vy-py*(vx*vx+vz*vz)+pz*vy*vz)/(std::sqrt((vx*vx+vz*vz)*y*y-2*y*(vx*vy*x+vy*vz*z-px*vx*vy+py*(vx*vx+vz*vz)-pz*vy*vz)+(vy*vy+vz*vz)*x*x-2*x*(vx*vz*z+px*(vy*vy+vz*vz)-(py*vy+pz*vz)*vx)+(vx*vx+vy*vy)*z*z+2*(px*vx*vz+py*vy*vz-pz*(vx*vx+vy*vy))*z+px*px*(vy*vy+vz*vz)-2*px*(py*vy+pz*vz)*vx+py*py*(vx*vx+vz*vz)-2*py*pz*vy*vz+pz*pz*(vx*vx+vy*vy))));
            //float dz = (((vx*vx+vy*vy)*z-vx*vz*x-vy*vz*y+px*vx*vz+py*vy*vz-pz*(vx*vx+vy*vy))/(std::sqrt((vx*vx+vy*vy)*z*z-2*z*(vx*vz*x+vy*vz*y-px*vx*vz-py*vy*vz+pz*(vx*vx+vy*vy))+(vy*vy+vz*vz)*x*x-2*x*(vx*vy*y+px*(vy*vy+vz*vz)-(py*vy+pz*vz)*vx)+(vx*vx+vz*vz)*y*y+2*(px*vx*vy-py*(vx*vx+vz*vz)+pz*vy*vz)*y+px*px*(vy*vy+vz*vz)-2*px*(py*vy+pz*vz)*vx+py*py*(vx*vx+vz*vz)-2*py*pz*vy*vz+pz*pz*(vx*vx+vy*vy))));
            float mult = ((1)/((vx*vx+vy*vy+vz*vz)))*(i+1)/frames.size();//std::sqrt
            float dx = 2*(vy*vy+vz*vz)*x-2*vx*vy*y-2*vx*vz*z+px*(-2*vy*vy-2*vz*vz)+2*py*vx*vy+2*pz*vx*vz;
            float dy = 2*(vx*vx+vz*vz)*y-2*vx*vy*x-2*vy*vz*z+2*px*vx*vy+py*(-2*vx*vx-2*vz*vz)+2*pz*vy*vz;
            float dz = 2*(vx*vx+vy*vy)*z-2*vx*vz*x-2*vy*vz*y+2*px*vx*vz+2*py*vy*vz+pz*(-2*vx*vx-2*vy*vy);
            
            //if (dx!=dx){
            //    dx = 0;
            //}
            //if (dy!=dy){
            //    dy = 0;
            //}
            //if (dz!=dz){
            //    dz = 0;
            //}
            dv -= {dx*mult,dy*mult,dz*mult};
        }
        positions[p] += dv/frames.size()*0.5;
    }
}

void pointSolve(std::vector<vector3D>& positions, const std::vector<instance>& frames) {
    if (frames.size()<2) return;
    for (int p = 0; p < positions.size(); p++){   
        instance i1 = frames[frames.size()-1];
        instance i2 = frames[frames.size()-2];
        vector3D p1 = i1.loc();
        vector3D p2 = i2.loc();

        vector3D d = p1-p2;
        vector3D a = i1.dir(p);
        vector3D b = i2.dir(p);

        float v1m = ((a).dot(b)*(b).dot(d)-(b).dot(b)*(a).dot(d))/((a).dot(a)*(b).dot(b)-(a).dot(b)*(a).dot(b));
        float v2m = ((a).dot(b)*(a).dot(d)-(a).dot(a)*(b).dot(d))/((a).dot(a)*(b).dot(b)-(a).dot(b)*(a).dot(b));
        vector3D est = (p1+p2+a*v1m+b*v2m)/2;
        positions[p] = est;
    }
}

void adjust3DpointsToAllignWithTheCamera(std::vector<vector3D>& positions, const std::vector<vector2D>& features, DroneState previousState) {
    if (positions.size() != features.size()) std::cout << "positions and their coresponding positions on camera do not match\n";
    drone temp = {previousState};
    //std::vector<vector2D> renderPoints = temp.render(positions);
    //for (int i = 0; i < positions.size(); i++){
    //    vector3D distance = positions[i]-previousState.loc;
    //    vector2D diff = solvePointOnPlane(previousState.forwardRot(),previousState.rightRot(),previousState.downRot(),distance);
    //    vector3D distanceIMG = previousState.forwardRot()+previousState.downRot()*features[i].y+previousState.rightRot()*features[i].x;
    //    float scale = std::sqrt(distance.dot(distance))/std::sqrt(distanceIMG.dot(distanceIMG));
    //    positions[i] -= previousState.rightRot() *(diff.x-features[i].x)*scale;
    //    positions[i] -= previousState.downRot()  *(diff.y-features[i].y)*scale;
    //}
    positions = temp.estimate3DPositions(features);
}