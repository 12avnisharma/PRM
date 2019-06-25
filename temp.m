Q=[1 0 heuristic(vertex(1,:),goal) 0+heuristic(vertex(1,:),goal) -1]; 
closed=[]; 
pathFound=false;
tic;
while size(Q,1)>0
     [A, I]=min(Q,[],1);
     n=Q(I(4),:); 
     Q=[Q(1:I(4)-1,:);Q(I(4)+1:end,:)]; 
     if n(1)==2
         pathFound=true;break;
     end
     for mv=1:length(edges{n(1),1}) 
         newVertex=edges{n(1),1}(mv);
         if length(closed)==0 || length(find(closed(:,1)==n(1)))==0 
             historicCost=n(2)+historic(vertex(n(1),:),vertex(newVertex,:));
             heuristicCost=heuristic(vertex(newVertex,:),goal);
             totalCost=historicCost+heuristicCost;
             add=true; 
             if length(find(Q(:,1)==n(1)))>=1
                 I=find(Q(:,1)==newVertex);
%                  if Q(I,1)==48 && n(1)==46 123, end
                 if Q(I,4)<totalCost, add=false;
                 else Q=[Q(1:I-1,:);Q(I+1:end,:);];add=true;
                 end
             end
             if add
                 Q=[Q;newVertex historicCost heuristicCost totalCost size(closed,1)+1]; 
         end           
     end
     closed=[closed;n]; 
end