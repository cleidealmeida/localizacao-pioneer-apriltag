function iFlags(tag)
% iFlags: : Initializes internal status flags for the AprilTag object.
% These flags are used to monitor the connectivity and operational state 
% of the system during tasks or simulations.

% --- Status Flags ---
    tag.pFlag.Connected  = 0; % Hardware connection status
    tag.pFlag.Detected   = false; % Vision detection status
    tag.pFlag.CADLoaded  = 0; % CAD data existence
    tag.pFlag.CADCreated = 0; % Graphics handle existence (patch)
end