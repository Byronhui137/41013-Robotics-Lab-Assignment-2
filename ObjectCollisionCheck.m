function inObjectCollision = ObjectCollisionCheck(startPoints, endPoints, vertex, faces, faceNormals) %Check for collision with laser curtain using line triangle intersection
    inObjectCollision = 0; %Set collision flag to 0
    for i = 1 : size(startPoints,1) % Check Top Panel laser curtain only to reduce computations 
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,startPoints(i,:),endPoints(i,:)); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:)) %If line intersects with plane and intersection points is inside triangle then return 1
                inObjectCollision = 1; % Return 1 if rectangle is in collision with laser curtain
            end
        end    
    end
end    
    
 function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts) 

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end