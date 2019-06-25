map=im2bw(imread('map1.bmp')); 
source=[10 10];
goal=[490 490];
n=50; 
display=true; 
if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k')
vertex=[source;goal]; 
if display, rectangle('Position',[vertex(1,2)-5,vertex(1,1)-5,10,10],'Curvature',[1,1],'FaceColor','r'); end
if display, rectangle('Position',[vertex(2,2)-5,vertex(2,1)-5,10,10],'Curvature',[1,1],'FaceColor','r'); end
tic;
while length(vertex)<n+2 
    x=double(int32(rand(1,2) .* size(map)));
    if feasiblePoint(x,map), 
        vertex=[vertex;x]; 
        if display, rectangle('Position',[x(2)-5,x(1)-5,10,10],'Curvature',[1,1],'FaceColor','r'); end
    end
end
if display 
    disp('click/press any key');
    waitforbuttonpress; 
end
edges=cell(n+2,1); 
for i=1:n+2
    for j=i+1:n+2
        if checkPath(vertex(i,:),vertex(j,:),map);
            edges{i}=[edges{i};j];edges{j}=[edges{j};i];
            if display, line([vertex(i,2);vertex(j,2)],[vertex(i,1);vertex(j,1)]); end
        end
    end
end
if display 
    disp('click/press any key');
    waitforbuttonpress; 
end
    
Q=[1 0 heuristic(vertex(1,:),goal) 0+heuristic(vertex(1,:),goal) -1]; 
closed=[];
pathFound=false;
while size(Q,1)>0
     [A, I]=min(Q,[],1);
     n=Q(I(4),:); 
     Q=[Q(1:I(4)-1,:);Q(I(4)+1:end,:)]; 
     if n(1)==2
         pathFound=true;break;
     end
     for mv=1:length(edges{n(1),1})
         newVertex=edges{n(1),1}(mv);
         if length(closed)==0 || length(find(closed(:,1)==newVertex))==0
             historicCost=n(2)+historic(vertex(n(1),:),vertex(newVertex,:));
             heuristicCost=heuristic(vertex(newVertex,:),goal);
             totalCost=historicCost+heuristicCost;
             add=true;
             if length(find(Q(:,1)==newVertex))>=1
                 I=find(Q(:,1)==newVertex);
                 if Q(I,4)<totalCost, add=false;
                 else Q=[Q(1:I-1,:);Q(I+1:end,:);];add=true;
                 end
             end
             if add
                 Q=[Q;newVertex historicCost heuristicCost totalCost size(closed,1)+1]; % add new nodes in queue
             end
         end           
     end
     closed=[closed;n]; 
end
if ~pathFound
    error('no path found')
end

fprintf('processing time=%d \nPath Length=%d \n\n', toc,n(4)); 
path=[vertex(n(1),:)]; 
prev=n(5);
while prev>0
    path=[vertex(closed(prev,1),:);path];
    prev=closed(prev,5);
end

imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k')
line(path(:,2),path(:,1),'color','r');