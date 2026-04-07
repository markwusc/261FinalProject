%% Cabin sizing and payload mass estimation


%% Cabin sizing section

%Passenger Config
aisleWidth = 0.4; %m, approximately 16in, FAA Requirement is 15in
seatWidth = 0.45; %m, approximately 17.7in seat width, roughly similar to 737 seat width
seatPitch = 0.82; %m, approximately 32in seat pitch
rows = 4;         %NOTE: Last row will be include lavatory + jump seat for FA
seatsPerRow = 3;  % 1-2 configeration similar to the E140 series of regional jets

%Cargo config
    %Insert stuff on that here

%Medical config
    %Insert stuff on that here

cabinWidth = aisleWidth + seatWidth * seatsPerRow;
cabinLength = seatPitch * rows;
cabinHeight = 2; %2m cabin height is relatively standard

fprintf('cabinWidth = %.3f m\n', cabinWidth);
fprintf('cabinLength = %.3f m\n', cabinLength);


%% Cabin weight section (including sections)

% Selected seat Recaro BL3530
seatWeight = 10; %kg
totalSeatWeight = seatWeight * rows * seatsPerRow;

passengerWeightlb = 9 * (200 + 50) + 400; %lb, must convert to kg, includes checked baggage
passengerWeight = passengerWeightlb * 0.453592;

totalSeatWeigt = totalSeatWeight + passengerWeight;




fprintf('Total passenger payload = %.2f kg\n', totalSeatWeight + passengerWeight);