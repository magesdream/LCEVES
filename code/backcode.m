            %经过点评价
%             passbys = [passbys ,stpath{n,pop(m,n)}]; 
            
%             for jamN = 1:length(stAllpath{n}{num})-2
%                 p1 = stAllpath{n}{num}(jamN);
%                 p2 = stAllpath{n}{num}(jamN+1);
%                 tj1 = trafficJamArr(:,1);
%                 tj2 = trafficJamArr(:,2);
%                 tj = tj1.*tj2;
%                 tjtag = find(tj==p1*p2);
%                 if (tjtag>0)
%                     trafficJamArr(tjtag,3)=trafficJamArr(tjtag,3)+1;
%                 else
%                     trafficJamArr(end+1,:) = [p1,p2,1];
%                 end
%             end