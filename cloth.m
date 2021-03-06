clf;
clc;
clear all;
    

n_iter = 4000;
    
S.h = plot3(0, 0,0);
S.mText = uicontrol('style','text');

xlabel('xmeter');
ylabel('ymeter');
zlabel('zmeter');

row = 10;
col = 10;
stiffness = 10;    % N/m
damping =  1;      % Ns/m
mass = 0.01;        % Kg
ts = 0.005;         % Seconds
    
% Build the nodes
nodes = buildNodes(row, col);
canvas = createCanvas(nodes);
canvas = drawNodes(S, canvas, nodes, 0);
    

for i = 0 : n_iter
    nodes = updateNode(nodes, mass, stiffness, damping, ts);
    if mod(i, 10) == 0
        canvas = drawNodes(S, canvas, nodes, ts*i);
    end
end
        


%%
function nodes = buildNodes(row, col)
% Build the nodes 
    nodes.row = row;
    nodes.col = col;
    
    for c = 1: col
        for r = 1 : row
            node(r,c).initalPos = [(c - 1) / 10 0 (r - 1) / 10]; % 1 cm step
            node(r,c).pos = node(r,c).initalPos;
            node(r,c).acc = [0 0 0];
            node(r,c).vel = [rand rand rand];
            node(r,c).norm = [0 0 0];
            % The last row is fixed
            if (c == 1) 
                node(r,c).isFixed = 1;
            else
                node(r,c).isFixed = 0;
            end
        end
    end
    
    nodes.node = node;
end

%% 
function nodes = updateNode(nodes, mass, stiffness, damping, ts)
% Update all nodes per time sampling
    row = nodes.row;
    col = nodes.col;
    node = nodes.node;
    wind_vc = 1;
    p = pi*ts;
    wind_speed = [200 200 0];
    
    % Force update
    % Calculate force on each node
    for r = 1 : row
        nextRow = r + 1;
        prevRow = r - 1;
        
        for c = 1 : col
            nextCol = c + 1;
            prevCol = c - 1;
            
            f1 = [0 0 0];
            f2 = [0 0 0];
            f3 = [0 0 0];
            f4 = [0 0 0];
            f5 = [0 0 0];
            f6 = [0 0 0];
            f7 = [0 0 0];
            f8 = [0 0 0];

            % Link 1
            if (r < row && c > 1)
                l0 = node(r, c).initalPos - node(nextRow, prevCol).initalPos;
                lt = node(r, c).pos - node(nextRow, prevCol).pos;
                n = norm(lt, 2);                
                f1 = stiffness * (n - norm(l0, 2)) * lt / n;
            end

            % Link 2
            if (r < row)
                l0 = node(r, c).initalPos - node(nextRow, c).initalPos;
                lt = node(r, c).pos - node(nextRow, c).pos;
                n = norm(lt, 2);
                f2 = stiffness * (n - norm(l0, 2)) * lt / n;
            end

            % Link 3
            if (c < col)
                l0 = node(r, c).initalPos - node(r, nextCol).initalPos;
                lt = node(r, c).pos - node(r, nextCol).pos;
                n = norm(lt, 2);
                f3 = stiffness * (n - norm(l0, 2)) * lt / n;
            end

            % Link 4
            if (r > 1 && c < col)
                l0 = node(r, c).initalPos - node(prevRow, nextCol).initalPos;
                lt = node(r, c).pos - node(prevRow, nextCol).pos;
                n = norm(lt, 2);
                f4 = stiffness * (n - norm(l0, 2)) * lt / n;
            end

            % Link 5
            if (r > 1)
                l0 = node(r, c).initalPos - node(prevRow, c).initalPos;
                lt = node(r, c).pos - node(prevRow, c).pos;
                n = norm(lt, 2);
                f5 = stiffness * (n - norm(l0, 2)) * lt / n;    
            end

            % Link 6
            if (c > 1)
                l0 = node(r, c).initalPos - node(r, prevCol).initalPos;                        
                lt = node(r, c).pos - node(r, prevCol).pos; 
                n = norm(lt, 2);
                f6 = stiffness * (n - norm(l0, 2)) * lt / n;                     
            end
            
            % Link 7
            if (r < row && c < col)
                l0 = node(r, c).initalPos - node(nextRow, nextCol).initalPos;                        
                lt = node(r, c).pos - node(nextRow, nextCol).pos; 
                n = norm(lt, 2);
                f7 = stiffness * (n - norm(l0, 2)) * lt / n;                     
            end
            
            % Link 8
            if (r > 1 && c > 1)
                l0 = node(r, c).initalPos - node(prevRow, prevCol).initalPos;                        
                lt = node(r, c).pos - node(prevRow, prevCol).pos; 
                n = norm(lt, 2);
                f8 = stiffness * (n - norm(l0, 2)) * lt / n;                     
            end
            
            %normal of point
           
            if (r ~= 1 && r ~= row)
                if (c == 1)
                    node(r,c).norm = cross((node(r, c+1).pos-node(r, c).pos),(node(r+1, c).pos-node(r-1, c).pos));
                elseif (c == row)
                    node(r,c).norm = cross((node(r, c).pos-node(r, c-1).pos),(node(r+1, c).pos-node(r-1, c).pos)); 
                else
                    node(r,c).norm = cross((node(r, c+1).pos-node(r, c-1).pos),(node(r+1, c).pos-node(r-1, c).pos));
                end
            end
            
            if (c ~= 1 && c ~= col)
                if (r == 1)
                    node(r,c).norm = cross((node(r, c+1).pos-node(r, c-1).pos),(node(r+1, c).pos-node(r, c).pos));
                elseif (r == row)
                    node(r,c).norm = cross((node(r, c+1).pos-node(r, c-1).pos),(node(r, c).pos-node(r-1, c).pos)); 
                end
            end
            
            if (c == 1 && r == 1)
                node(r,c).norm = cross((node(r, c+1).pos-node(r, c).pos),(node(r+1, c).pos-node(r, c).pos));
            end
            if (c == 1 && r == row)
                node(r,c).norm = cross((node(r, c+1).pos-node(r, c).pos),(node(r, c).pos-node(r-1, c).pos));
            end
            if (c == col && r == row)
                node(r,c).norm = cross((node(r, c).pos-node(r, c-1).pos),(node(r, c).pos-node(r-1, c).pos));
            end
            if (c == col && r == 1)
                node(r,c).norm = cross((node(r, c).pos-node(r, c-1).pos),(node(r+1, c).pos-node(r, c).pos));
            end
            
            
            node(r,c).force =  -f1 - f2 - f3 - f4 - f5 - f6 - f7 - f8 - ... 
                               damping * node(r,c).vel + mass * [0 0 -9.81] ...
                               + wind_vc*(dot(node(r,c).norm,(wind_speed-node(r,c).vel)))*node(r,c).norm;

        end
    end

    % Position, velocity, and acceelleration update
    for r = 1 : row        
        for c = 1: col
            if  node(r,c).isFixed ~= 1              
                node(r,c).acc = node(r,c).force ./ mass;
                node(r,c).vel = node(r,c).vel + node(r,c).acc .* ts;
                node(r,c).pos = node(r,c).pos + node(r,c).vel .* ts;    
            end               
        end
    end
    
    nodes.node = node;
end

%%
function canvas = createCanvas(nodes)
    % Graphic thingy     
    index = 1;
    for c = 1 : nodes.col
        for r = 1 : nodes.row
            canvas(index,:) = nodes.node(r, c).pos;
            index = index + 1;
        end
    end

    canvas_min = min(canvas);
    canvas_max = max(canvas);
    range = canvas_max - canvas_min;

    xlim([-1 1.5])
    ylim([-1 1])
    zlim([-1 2])
end

%% 
function canvas = drawNodes(S, canvas, nodes, timestamp)    
% Draw the nodes
    index = 1;    
    for c = 1 : nodes.col
        % Vertical line
        for r = nodes.row : -1 : 1
            canvas(index, :) = nodes.node(r, c).pos;
            index = index + 1;
        end

        % Zig-zag line
        for r = 1 : nodes.row
            canvas(index,:) = nodes.node(r,c).pos;
            index = index + 1;
            if (c < nodes.col)
                canvas(index ,:) = nodes.node(r, c + 1).pos;
                index = index + 1;
            end                          
        end

    end

    set(S.h, 'XData', canvas(:,1));
    set(S.h, 'YData', canvas(:,2));
    set(S.h, 'ZData', canvas(:,3));
    set(S.mText,'String', timestamp);

    drawnow;
end

