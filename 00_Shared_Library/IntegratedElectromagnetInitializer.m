classdef IntegratedElectromagnetInitializer

    methods(Static)

        % Following properties of 'maskInitContext' are available to use:
        %  - BlockHandle
        %  - MaskObject
        %  - MaskWorkspace: Use get/set APIs to work with mask workspace.
        function MaskInitialization(maskInitContext)
            % 1. Load global parameters into the block scope
            % This ensures N_coil, Km, etc., are available to the internal blocks.
                ElectromagnetConfig.assign_to_blocks(gcb);

                % Get the current mask parameter value
                maskObj = Simulink.Mask.get(gcb);
                initAirgapParam = maskObj.getParameter('l_airgap');
                initAirgapValue = str2double(initAirgapParam.Value);  % Convert to double if stored as string
 
                % Get the minimal safe air gap from the mask workspace
                minSafeAirgap = ElectromagnetConfig.min_safe_airgap;
                if initAirgapValue <= minSafeAirgap
                    initAirgapParam.Value = num2str(minSafeAirgap);
                   % initAirgapParam.Value = num2str(max(minSafeAirgap, initAirgapValue));
                end
        end

        % Use the code browser on the left to add the callbacks.
        
        
    end
end

