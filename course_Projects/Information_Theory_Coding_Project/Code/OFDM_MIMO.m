%% Project of Information Theory and Coding (ECE4007)
%% Project Title: MIMO-OFDM Phased Array Precoding

%% Description
% This project shows how phased arrays are used in a MIMO-OFDM
% communication system employing beamforming. Using components from Phased
% Array System Toolbox(TM), this project models the radiating elements that
% comprise a transmitter and the front-end receiver components, for a
% MIMO-OFDM communication system. With user-specified parameters, the
% designed allows you to validate the performance of the system in terms of
% bit error rate and constellations for different spatial locations and
% array sizes.



s = rng(61); % Set RNG state for repeatability

%% System Parameters

% Single-user system with multiple streams
prm.numUsers = 1; % No.of Users
prm.numSTS = 16; % No.of Data Streams
prm.numTx = 32;              % Number of transmit antennas 
prm.numRx = 16;              % Number of receive antennas 
prm.bitsPerSubCarrier = 6;   % 2: QPSK, 4: 16QAM, 6: 64QAM, 8: 256QAM
prm.numDataSymbols = 10;     % Number of OFDM data symbols

prm.fc = 4e9;                   % 4 GHz system
prm.chanSRate = 100e6;          % Channel sampling rate, 100 Msps
prm.ChanType = 'WINNER';        % Channel options: 'WINNER', 'Scattering',
                                %           'ScatteringFcn', 'StaticFlat'
prm.NFig = 5;                   % Noise figure, dB

% Array locations and angles
prm.posTx = [0;0;0];            % BS/Transmit array position, [x;y;z], meters
prm.mobileRange = 300;          % meters
% Angles specified as [azimuth;elevation], az=[-90 90], el=[-90 90]
prm.mobileAngle = [33; 0];      % degrees
prm.steeringAngle = [30; -20];  % Transmit steering angle (close to mobileAngle)
prm.enSteering = true;          % Enable/disable steering




%%
% Parameters to define the OFDM modulation employed

prm.FFTLength = 256; 
prm.CyclicPrefixLength = 64; 
prm.numCarriers = 234; 
prm.NumGuardBandCarriers = [7 6];
prm.PilotCarrierIndices = [26 54 90 118 140 168 204 232];
nonDataIdx = [(1:prm.NumGuardBandCarriers(1))'; prm.FFTLength/2+1; ...
              (prm.FFTLength-prm.NumGuardBandCarriers(2)+1:prm.FFTLength)'; ...
              prm.PilotCarrierIndices.';];
prm.CarriersLocations = setdiff((1:prm.FFTLength)',sort(nonDataIdx));

numTx = prm.numTx;
numRx = prm.numRx;
numSTS = prm.numSTS;
prm.numFrmBits = numSTS*prm.numDataSymbols*prm.numCarriers* ...
                 prm.bitsPerSubCarrier*1/3-6; % Account for termination bits

prm.modMode = 2^prm.bitsPerSubCarrier; % Modulation order
% Account for channel filter delay
prm.numPadZeros = 3*(prm.FFTLength+prm.CyclicPrefixLength); 

% Get transmit and receive array information
prm.numSTSVec = numSTS;
[isTxURA,expFactorTx,isRxURA,expFactorRx] = helperArrayInfo(prm,true);

%%
% Using utilities from the Phased Array System Toolbox, the free space path
% loss is calculated based on the base station and mobile station positions
% for the spatially-aware system modeled.

prm.cLight = physconst('LightSpeed');
prm.lambda = prm.cLight/prm.fc;
% Mobile position
[xRx,yRx,zRx] = sph2cart(deg2rad(prm.mobileAngle(1)),...
                         deg2rad(prm.mobileAngle(2)),prm.mobileRange);
prm.posRx = [xRx;yRx;zRx];
[toRxRange,toRxAng] = rangeangle(prm.posTx,prm.posRx);
spLoss = fspl(toRxRange,prm.lambda);
gainFactor = 1;

%% Channel Sounding
%
% For a spatially multiplexed system, availability of channel information
% at the transmitter allows for precoding to be applied to maximize
% the signal energy in the direction and channel of interest. Under the
% assumption of a slowly varying channel, this is facilitated by sounding
% the channel first, wherein for a reference transmission, the receiver
% estimates the channel and feeds this information back to the transmitter.
%
% For the chosen system, a preamble signal is sent over all transmitting
% elements, and processed at the receiver accounting for the channel. The
% receiver elements perform pre-amplification, OFDM demodulation, frequency
% domain channel estimation, and calculation of the feedback weights based
% on channel diagonalization using singular value decomposition (SVD)
% per data subcarrier. 

% Generate the preamble signal
preambleSigSTS = helperGenPreamble(prm);
%   repeat over numTx
preambleSig = zeros(size(preambleSigSTS,1),numTx);
for i = 1:numSTS
    preambleSig(:,(i-1)*expFactorTx+(1:expFactorTx)) = ...
        repmat(preambleSigSTS(:,i),1,expFactorTx);
end

% Transmit preamble over channel
[rxPreSig,chanDelay] = helperApplyChannel(preambleSig,prm,spLoss);

% Front-end amplifier gain and thermal noise
rxPreAmp = phased.ReceiverPreamp( ...
    'Gain',gainFactor*spLoss, ... % account for path loss
    'NoiseFigure',prm.NFig, ...
    'ReferenceTemperature',290, ...
    'SampleRate',prm.chanSRate);
rxPreSigAmp = rxPreAmp(rxPreSig);
rxPreSigAmp = rxPreSigAmp*(sqrt(prm.FFTLength- ... % scale power
              sum(prm.NumGuardBandCarriers)-1)/(prm.FFTLength));  

% OFDM Demodulation
demodulatorOFDM = comm.OFDMDemodulator( ...
     'FFTLength',prm.FFTLength, ...
     'NumGuardBandCarriers',prm.NumGuardBandCarriers.', ...
     'RemoveDCCarrier',true, ...
     'PilotOutputPort',true, ...
     'PilotCarrierIndices',prm.PilotCarrierIndices.', ...
     'CyclicPrefixLength',prm.CyclicPrefixLength, ...
     'NumSymbols',numSTS, ... % preamble symbols alone
     'NumReceiveAntennas',numRx);

rxOFDM = demodulatorOFDM( ...
    rxPreSigAmp(chanDelay+1:end-(prm.numPadZeros-chanDelay),:));

% Channel estimation from preamble
%       numCarr, numSTS, numRx
hD = helperMIMOChannelEstimate(rxOFDM(:,1:numSTS,:),prm); 

% Calculate the feedback weights
v = diagbfweights(hD);

%% 
% For conciseness in presentation, front-end synchronization including
% carrier and timing recovery are assumed. The weights computed using
% |diagbfweights| are hence fed back to the transmitter, for subsequent
% application for the actual data transmission.

%% Data Transmission 
%
% Next, we configure the system's data transmitter. This processing
% includes channel coding, bit mapping to complex symbols, splitting of the
% individual data stream to multiple transmit streams, precoding of the
% transmit streams, OFDM modulation with pilot mapping and replication for
% the transmit antennas employed.

% Convolutional encoder
encoder = comm.ConvolutionalEncoder( ...
    'TrellisStructure',poly2trellis(7,[133 171 165]), ...
    'TerminationMethod','Terminated');

% Bits to QAM symbol mapping
modulatorRQAM = comm.RectangularQAMModulator( ...
    'ModulationOrder',prm.modMode, ...
    'BitInput',true, ...
    'NormalizationMethod','Average power');

% Generate mapped symbols from bits
txBits = randi([0, 1],prm.numFrmBits,1);
encodedBits = encoder(txBits);
mappedSym = modulatorRQAM(encodedBits);

% Map to layers: per symbol, per data stream
gridData = reshape(mappedSym,prm.numCarriers,prm.numDataSymbols,numSTS);

% Apply precoding weights to the subcarriers, assuming perfect feedback
preData = complex(zeros(prm.numCarriers,prm.numDataSymbols,numSTS));
for symIdx = 1:prm.numDataSymbols
    for carrIdx = 1:prm.numCarriers
        Q = squeeze(v(carrIdx,:,:));
        normQ = Q * sqrt(numTx)/norm(Q,'fro');      
        preData(carrIdx,symIdx,:) = squeeze(gridData(carrIdx,symIdx,:)).' ...
            * normQ;
    end
end

% OFDM modulation of the data
modulatorOFDM = comm.OFDMModulator( ...
    'FFTLength',prm.FFTLength,...
    'NumGuardBandCarriers',prm.NumGuardBandCarriers.',...
    'InsertDCNull',true, ...
    'PilotInputPort',true,...
    'PilotCarrierIndices',prm.PilotCarrierIndices.',...
    'CyclicPrefixLength',prm.CyclicPrefixLength,...
    'NumSymbols',prm.numDataSymbols,...
    'NumTransmitAntennas',numSTS);

% Multi-antenna pilots
pilots = helperGenPilots(prm.numDataSymbols,numSTS);

txOFDM = modulatorOFDM(preData,pilots);
txOFDM = txOFDM * (prm.FFTLength/ ...
    sqrt(prm.FFTLength-sum(prm.NumGuardBandCarriers)-1)); % scale power

% Generate preamble with the feedback weights and prepend to data
preambleSigD = helperGenPreamble(prm,v);
txSigSTS = [preambleSigD;txOFDM];

% Repeat over numTx
txSig = zeros(size(txSigSTS,1),numTx);
for i = 1:numSTS
    txSig(:,(i-1)*expFactorTx+(1:expFactorTx)) = ...
        repmat(txSigSTS(:,i),1,expFactorTx);
end

%%
% For precoding, the preamble signal is regenerated to enable proper
% channel estimation. It is prepended to the data portion to form the
% transmission packet which is then replicated over the transmit antennas.

%% Transmit Beam Steering 
%
% Phased Array System Toolbox offers components appropriate for the design
% and simulation of phased arrays used in wireless communications systems.
%
% For the spatially aware system, the signal transmitted from the base
% station is steered towards the direction of the mobile, so as to focus
% the radiated energy in the desired direction. This is achieved by
% applying a phase shift to each antenna element to steer the transmission.
%
% The project uses a linear or rectangular array at the transmitter,
% depending on the number of data streams and number of transmit antennas
% selected.

% Gain per antenna element 
amplifier = phased.Transmitter('PeakPower',1/numTx,'Gain',0);

% Amplify to achieve peak transmit power for each element
for n = 1:numTx
    txSig(:,n) = amplifier(txSig(:,n));
end

% Transmit antenna array definition 
if isTxURA
    % Uniform Rectangular array
    arrayTx = phased.URA([expFactorTx,numSTS],[0.5 0.5]*prm.lambda, ...
        'Element',phased.IsotropicAntennaElement('BackBaffled',true));
else
    % Uniform Linear array
    arrayTx = phased.ULA(numTx, ...
        'ElementSpacing',0.5*prm.lambda, ...
        'Element',phased.IsotropicAntennaElement('BackBaffled',true));
end

% For evaluating weights for steering  
SteerVecTx = phased.SteeringVector('SensorArray',arrayTx, ...
    'PropagationSpeed',prm.cLight);

% Generate weights for steered direction
wT = SteerVecTx(prm.fc,prm.steeringAngle);

% Radiate along the steered direction, without signal combining
radiatorTx = phased.Radiator('Sensor',arrayTx, ...
    'WeightsInputPort',true, ...
    'PropagationSpeed',prm.cLight, ...
    'OperatingFrequency',prm.fc, ...
    'CombineRadiatedSignals',false);

if prm.enSteering
    txSteerSig = radiatorTx(txSig,repmat(prm.mobileAngle,1,numTx), ...
        conj(wT));
else
    txSteerSig = txSig;
end

% Visualize the array
h = figure('Position',figposition([10 55 22 35]),'MenuBar','none');
h.Name = 'Transmit Array Geometry';
viewArray(arrayTx);

% Visualize the transmit pattern and steering
h = figure('Position',figposition([32 55 22 30]));
h.Name = 'Transmit Array Response Pattern';
pattern(arrayTx,prm.fc,'PropagationSpeed',prm.cLight,'Weights',wT);
h = figure('Position',figposition([54 55 22 35]),'MenuBar','none');
h.Name = 'Transmit Array Azimuth Pattern';
patternAzimuth(arrayTx,prm.fc,'PropagationSpeed',prm.cLight,'Weights',wT);
if isTxURA
    h = figure('Position',figposition([76 55 22 35]),'MenuBar','none');
    h.Name = 'Transmit Array Elevation Pattern';
    patternElevation(arrayTx,prm.fc,'PropagationSpeed',prm.cLight, ...
        'Weights',wT);
end

%% 
% The plots indicate the array geometry and the transmit array response in
% multiple views. The response shows the transmission direction as
% specified by the steering angle.
%
% In this project we assume the steering angle known and close to the mobile
% angle. In actual systems, this would be estimated from angle-of-arrival
% estimation at the receiver as a part of the channel sounding or initial
% beam tracking procedures.

%% Signal Propagation 
%
% The project offers three options for spatial MIMO channels and a
% simpler static-flat MIMO channel for evaluation purposes.
%
% The WINNER II channel model [ <#19 5> ] is a spatially defined MIMO
% channel that allows you to specify the array geometry and location
% information. The project uses the typical urban microcell indoor scenario
% with very low mobile speeds.
%
% The two scattering models use a single-bounce ray tracing approximation
% where the number of scatterers is user-specified. For this project, the
% number of scatterers is set to 100. The 'Scattering' option models the
% scatterers placed randomly within a circle in between the transmitter and
% receiver, while the 'ScatteringFcn' models their placement completely
% randomly.
%
% The models allow path loss modeling and both line-of-sight (LOS) and
% non-LOS propagation conditions. The project assumes non-LOS propagation
% and isotropic antenna element patterns with linear geometry.

% Apply a spatially defined channel to the steered signal
[rxSig,chanDelay] = helperApplyChannel(txSteerSig,prm,spLoss,preambleSig);

%% 
% The same channel is used for both sounding and data transmission, with
% the data transmission having a longer duration controlled by the number
% of data symbols parameter, |prm.numDataSymbols|.

%% Receive Beam Steering
%
% The receiver steers the incident signals to align with the transmit end
% steering, per receive element. Thermal noise and receiver gain are
% applied. Uniform linear or rectangular arrays with isotropic responses
% are modeled to match the channel and transmitter arrays.

rxPreAmp = phased.ReceiverPreamp( ...
    'Gain',gainFactor*spLoss, ... % accounts for path loss
    'NoiseFigure',prm.NFig, ...
    'ReferenceTemperature',290, ...
    'SampleRate',prm.chanSRate);

% Front-end amplifier gain and thermal noise
rxSigAmp = rxPreAmp(rxSig);
rxSigAmp = rxSigAmp *(sqrt(prm.FFTLength- ...   % scale power
    sum(prm.NumGuardBandCarriers)-1)/(prm.FFTLength)); 

% Receive array
if isRxURA 
    % Uniform Rectangular array
    arrayRx = phased.URA([expFactorRx,numSTS],0.5*prm.lambda, ...
        'Element',phased.IsotropicAntennaElement('BackBaffled',true));
else 
    % Uniform Linear array
    arrayRx = phased.ULA(numRx, ...
        'ElementSpacing',0.5*prm.lambda, ...
        'Element',phased.IsotropicAntennaElement);
end
    
% For evaluating receive-side steering weights 
SteerVecRx = phased.SteeringVector('SensorArray',arrayRx, ...
    'PropagationSpeed',prm.cLight);

% Generate weights for steered direction towards mobile
wR = SteerVecRx(prm.fc,toRxAng);

% Steer along the mobile receive direction
if prm.enSteering
    rxSteerSig = rxSigAmp.*(wR');
else
    rxSteerSig = rxSigAmp;
end

% Visualize the array
h = figure('Position',figposition([10 20 22 35]),'MenuBar','none');
h.Name = 'Receive Array Geometry';
viewArray(arrayRx);

% Visualize the receive pattern and steering
h = figure('Position',figposition([32 20 22 30]));
h.Name = 'Receive Array Response Pattern';
pattern(arrayRx,prm.fc,'PropagationSpeed',prm.cLight,'Weights',wR);
h = figure('Position',figposition([54 20 22 35]),'MenuBar','none');
h.Name = 'Receive Array Azimuth Pattern';
patternAzimuth(arrayRx,prm.fc,'PropagationSpeed',prm.cLight,'Weights',wR);
if isRxURA 
    figure('Position',figposition([76 20 22 35]),'MenuBar','none');
    h.Name = 'Receive Array Elevation Pattern';
    patternElevation(arrayRx,prm.fc,'PropagationSpeed',prm.cLight, ...
        'Weights',wR);
end

%%
% The receive antenna pattern mirrors the transmission steering.

%% Signal Recovery
%
% The receive antenna array passes the propagated signal to the receiver to
% recover the original information embedded in the signal. Similar to the
% transmitter, the receiver used in a MIMO-OFDM system contains many
% stages, including OFDM demodulator, MIMO equalizer, QAM demodulator, and
% channel decoder.

demodulatorOFDM = comm.OFDMDemodulator( ...
     'FFTLength',prm.FFTLength, ...
     'NumGuardBandCarriers',prm.NumGuardBandCarriers.', ...
     'RemoveDCCarrier',true, ...
     'PilotOutputPort',true, ...
     'PilotCarrierIndices',prm.PilotCarrierIndices.', ...
     'CyclicPrefixLength',prm.CyclicPrefixLength, ...
     'NumSymbols',numSTS+prm.numDataSymbols, ... % preamble & data
     'NumReceiveAntennas',numRx);
  
% OFDM Demodulation
rxOFDM = demodulatorOFDM( ...
    rxSteerSig(chanDelay+1:end-(prm.numPadZeros-chanDelay),:));

% Channel estimation from the mapped preamble
hD = helperMIMOChannelEstimate(rxOFDM(:,1:numSTS,:),prm);

% MIMO Equalization
[rxEq,CSI] = helperMIMOEqualize(rxOFDM(:,numSTS+1:end,:),hD);

% Soft demodulation
scFact = ((prm.FFTLength-sum(prm.NumGuardBandCarriers)-1) ...
         /prm.FFTLength^2)/numTx;
nVar = noisepow(prm.chanSRate,prm.NFig,290)/scFact;
demodulatorRQAM = comm.RectangularQAMDemodulator( ...
     'ModulationOrder',prm.modMode, ...
     'BitOutput',true, ...
     'DecisionMethod','Approximate log-likelihood ratio', ...
     'NormalizationMethod','Average power', ...
     'Variance',nVar); 
rxSymbs = rxEq(:)/sqrt(numTx);
rxLLRBits = demodulatorRQAM(rxSymbs);

% Apply CSI prior to decoding
rxLLRtmp = reshape(rxLLRBits,prm.bitsPerSubCarrier,[], ...
                   prm.numDataSymbols,numSTS);
csitmp = reshape(CSI,1,[],1,numSTS);
rxScaledLLR = rxLLRtmp.*csitmp;

% Soft-input channel decoding
decoder = comm.ViterbiDecoder(...
     'InputFormat','Unquantized', ...
     'TrellisStructure',poly2trellis(7, [133 171 165]), ...
     'TerminationMethod','Terminated', ...
     'OutputDataType','double');
rxDecoded = decoder(rxScaledLLR(:));

% Decoded received bits
rxBits = rxDecoded(1:prm.numFrmBits);

%% 
% For the MIMO system modeled, the displayed receive constellation of the
% equalized symbols offers a qualitative assessment of the reception. The
% actual bit error rate offers the quantitative figure by comparing the
% actual transmitted bits with the received decoded bits.

% Display received constellation
constDiag = comm.ConstellationDiagram( ...
    'SamplesPerSymbol',1, ...
    'ShowReferenceConstellation',true, ...
    'ReferenceConstellation',constellation(modulatorRQAM), ...
    'ColorFading',false, ...
    'Position',figposition([20 20 35 40]), ...
    'Title','Equalized Symbols', ...
    'EnableMeasurements',true, ...
    'MeasurementInterval',length(rxSymbs));
constDiag(rxSymbs);

% Compute and display bit error rate
ber = comm.ErrorRate;
measures = ber(txBits,rxBits);
fprintf('BER = %.5f; No. of Bits = %d; No. of errors = %d\n', ...
    measures(1),measures(3),measures(2));

rng(s); % Restore RNG state

%% Conclusion and Further Exploration
% 
% The project highlighted the use of phased antenna arrays for a beamformed
% MIMO-OFDM system. It accounted for the spatial geometry and location of
% the arrays at the base station and mobile station for a single user
% system. Using channel sounding, the project illustrates how precoding is
% realized in current wireless systems and how steering of antenna arrays
% is modeled. 

