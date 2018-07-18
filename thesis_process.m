function varargout = thesis_process(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @thesis_process_OpeningFcn, ...
                   'gui_OutputFcn',  @thesis_process_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

function thesis_process_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;

guidata(hObject, handles);

function varargout = thesis_process_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;


% --- Executes on button press in importImage.
function importImage_Callback(hObject, eventdata, handles)

%Import Image
%error = imread('C:\Users\jesse\Desktop\error.jpg');
[a,b] = uigetfile({'*.png';'*.jpg'},'Pick an Image(.png,.jpg) File');
Original = imread([b a]); 
imshow(Original,'Parent',handles.originalImage);
angleFinal = 0;
angleFinalOld = 0;
%Convert Original to Grayscale
Grayscale = rgb2gray(Original);

%Binarization of Image
BW = imbinarize(Grayscale);

%Invert Image
BW2 = imcomplement(BW);
BW2 = bwareaopen(BW2,10);


%Old Algorithm
tic
try
[rows,cols] = size(BW2);

ctr = false;
for col = 1:cols
    for row = 1:rows
        if BW2(row,col) == 1
            if ctr == false
                leftmost = [row,col];
            ctr = true;
            end
        end
    end
end

ctr = false;
for row = 1:rows
    for col = 1:cols
        if BW2(row,col) == 1
            if ctr == false
                topmost = [row,col];
            ctr = true;
            end
        end
    end
end

fpX = topmost(2);
fpY = topmost(1);
spX = leftmost(2);
spY = leftmost(1);

halfc = col/2;

if topmost(2) < halfc
   rotation = 'cclockwise'; 
else
   rotation = 'clockwise';
end

if rotation == "cclockwise"
    slope = (spY - fpY)/(spX - fpX);
    angleNew = atan(slope);
    angleFinalOld = 90+(angleNew*180)/pi;
    angleFinalOlddisp = -angleFinalOld;
elseif rotation == "clockwise"
    slope = (spY - fpY)/(spX - fpX);
    angleNew = atan(slope);
    angleFinalOld = (angleNew*180)/pi;
    angleFinalOlddisp = -angleFinalOld;
end
rotatedOld = imrotate(BW2, angleFinalOld,'crop');

catch
    
end
oldAlgoRt = toc;

tic
try
%New algorithm

%Get centroids
dots = regionprops(BW2, Grayscale, 'all');
numberOfBlobs = size(dots, 1);
%disp(numberOfBlobs);

%Stored centroids of dots
allBlobCentroids = [dots.Centroid];
centroidsX = allBlobCentroids(1:2:end-1);
centroidsY = allBlobCentroids(2:2:end);
dataval=[centroidsX;centroidsY];
%disp(dataval);

%Get convex hull
initHull = convhull(centroidsX,centroidsY);

finalHull = initHull;
lastHull = length(initHull);
finalHull(lastHull) = [];
%disp(finalHull);

%Algorithm Start
ctr = 1;
counter  = true;
lengthHull = length(finalHull);

while counter == true
    %Initialization of first point
    if ctr == 1
        first = lengthHull;
    else
        first = ctr-1;
    end
    
    %Initialization of first point
    mid = ctr;

    %Initialization of last point
    if ctr == lengthHull
        last = 1;
    else
        last = ctr+1;
    end
    
    %Compute the degree between the three points
    if ctr > length(finalHull)
       break;
    end
    
    midP = [centroidsX(finalHull(mid)), centroidsY(finalHull(mid))];
    lastP = [centroidsX(finalHull(last)), centroidsY(finalHull(last))];
    firstP = [centroidsX(finalHull(first)), centroidsY(finalHull(first))];

    deg = atan2(abs(det([firstP-midP;lastP-midP])),dot(firstP-midP,lastP-midP));
    finalDeg = deg * (180/pi);
    %disp(finalDeg)
    
    %Condition if the midpoint is to be removed
    if finalDeg > 170 
        finalHull(mid) = [];
        lengthHull = length(finalHull);
    else
        ctr = ctr + 1;
    end
    
    if mid == lengthHull+1
        counter = false;
    end
end
finalHull(lengthHull+1) = finalHull(1);
finalHullCentroidsX = centroidsX(finalHull);
finalHullCentroidsY = centroidsY(finalHull);

ctrArray = 1;
maxPointsInLine = 0;

while ctrArray < length(finalHull)
    
    lineRecX = [finalHullCentroidsX(ctrArray)+20,finalHullCentroidsX(ctrArray)-20,finalHullCentroidsX(ctrArray+1)-50,finalHullCentroidsX(ctrArray+1)+50,finalHullCentroidsX(ctrArray)+50];
    lineRecY = [finalHullCentroidsY(ctrArray)+20,finalHullCentroidsY(ctrArray)-20,finalHullCentroidsY(ctrArray+1)-50,finalHullCentroidsY(ctrArray+1)+50,finalHullCentroidsY(ctrArray)+50];

    inside = inpolygon(centroidsX,centroidsY,lineRecX,lineRecY);
    numOfPoints = sum(inside(:) == 1);
    %disp(in);
    %disp(numOfPoints);
    
    if numOfPoints >= maxPointsInLine
        maxPointsInLine = numOfPoints;
        currentBasisLine = [finalHull(ctrArray),finalHull(ctrArray+1)];
    end
    
    ctrArray = ctrArray + 1;
end

slope = (centroidsY(currentBasisLine(2)) - centroidsY(currentBasisLine(1))) ./ (centroidsX(currentBasisLine(2)) - centroidsX(currentBasisLine(1)));
angleNew = atan(slope);
angleFinal = (angleNew*180)/pi;
angleFinaldisp = -angleFinal;
rotatedNew = imrotate(BW2, angleFinal,'crop');

catch 
   %imshow(error,'Parent',handles.new);
end
newAlgoRt = toc;
imshow(rotatedOld,'Parent',handles.old);
imshow(rotatedNew,'Parent',handles.new);
set(handles.angleOld, 'String', angleFinalOlddisp);
set(handles.angleNew, 'String', angleFinaldisp);
set(handles.rtOld, 'String', oldAlgoRt);
set(handles.rtNew, 'String', newAlgoRt);


% --- Executes on button press in reset.
function reset_Callback(hObject, eventdata, handles)
cla(handles.originalImage);
cla(handles.old);
cla(handles.new);
