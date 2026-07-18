function rCameraDisconnect(tag)
% rDisconnect: Safely releases and disconnects the associated camera hardware.
% This method ensures that the camera object is deleted from memory, freeing
% the hardware resource for other applications or future sessions.
%
% --- Mini Example ---
% % To manually release the camera before clearing the object:
%   tag.rDisconnect();
% % Output: Camera disconnected and released.
% ---------------------

% Check if there is an active camera object associated with the tag
if ~isempty(tag.pCam)
    % Delete the camera object to release the hardware resource
    delete(tag.pCam);

    % Confirmation message for the user
    disp('Camera disconnected and released.');

end
end