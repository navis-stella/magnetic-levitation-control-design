%
% Copyright (c) The acados authors.
%
% This file is part of acados.
%
% The 2-Clause BSD License
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright notice,
% this list of conditions and the following disclaimer in the documentation
% and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.;

%




  
  
  
  
  
  
  
  
  
  
  
  
  
  




SOURCES = { ...
            'MaglevAug_model/MaglevAug_expl_ode_fun.c', ...
            'MaglevAug_model/MaglevAug_expl_vde_forw.c', ...
            'MaglevAug_model/MaglevAug_expl_vde_adj.c', ...
            'MaglevAug_cost/MaglevAug_cost_y_0_fun.c', ...
            'MaglevAug_cost/MaglevAug_cost_y_0_fun_jac_ut_xt.c', ...
            'MaglevAug_cost/MaglevAug_cost_y_fun.c', ...
            'MaglevAug_cost/MaglevAug_cost_y_fun_jac_ut_xt.c', ...
            'MaglevAug_cost/MaglevAug_cost_y_e_fun.c', ...
            'MaglevAug_cost/MaglevAug_cost_y_e_fun_jac_ut_xt.c', ...
            'MaglevAug_constraints/MaglevAug_constr_h_0_fun.c', ...
            'MaglevAug_constraints/MaglevAug_constr_h_0_fun_jac_uxt_zt.c', ...
            'MaglevAug_constraints/MaglevAug_constr_h_fun.c', ...
            'MaglevAug_constraints/MaglevAug_constr_h_fun_jac_uxt_zt.c', ...
            'acados_solver_sfunction_MaglevAug.c', ...
            'acados_solver_MaglevAug.c'
          };

INC_PATH = 'C:\Optimization_Toolkit\acados\interfaces\acados_matlab_octave\..\../include';

INCS = {['-I', fullfile(INC_PATH, 'blasfeo', 'include')], ...
        ['-I', fullfile(INC_PATH, 'hpipm', 'include')], ...
        ['-I', fullfile(INC_PATH, 'acados')], ...
        ['-I', fullfile(INC_PATH)]};



CFLAGS = 'CFLAGS=$CFLAGS';
LDFLAGS = 'LDFLAGS=$LDFLAGS';
COMPFLAGS = 'COMPFLAGS=$COMPFLAGS';
COMPDEFINES = 'COMPDEFINES=$COMPDEFINES';



LIB_PATH = ['-L', fullfile('C:\Optimization_Toolkit\acados\interfaces\acados_matlab_octave\..\../lib')];

LIBS = {'-lacados', '-lhpipm', '-lblasfeo'};

% acados linking libraries and flags
LDFLAGS = [LDFLAGS ' '];
COMPFLAGS = [COMPFLAGS ' '];
LIBS{end+1} = '';
LIBS{end+1} = '';
LIBS{end+1} = '';

COMPFLAGS = [COMPFLAGS ' -O2'];
CFLAGS = [CFLAGS ' -O2'];

try
    %     mex('-v', '-O', CFLAGS, LDFLAGS, COMPFLAGS, COMPDEFINES, INCS{:}, ...
    mex('-O', CFLAGS, LDFLAGS, COMPFLAGS, COMPDEFINES, INCS{:}, ...
            LIB_PATH, LIBS{:}, SOURCES{:}, ...
            '-output', 'acados_solver_sfunction_MaglevAug' );
catch exception
    disp('make_sfun failed with the following exception:')
    disp(exception);
    disp(exception.message );
    disp('Try adding -v to the mex command above to get more information.')
    keyboard
end

fprintf( [ '\n\nSuccessfully created sfunction:\nacados_solver_sfunction_MaglevAug', '.', ...
    eval('mexext')] );


%% print note on usage of s-function, and create I/O port names vectors
fprintf('\n\nNote: Usage of Sfunction is as follows:\n')
input_note = 'Inputs are:\n';
i_in = 1;

global sfun_input_names
sfun_input_names = {};
input_note = strcat(input_note, num2str(i_in), ') lbx_0 - lower bound on x for stage 0,',...
                    ' size [3]\n ');
sfun_input_names = [sfun_input_names; 'lbx_0 [3]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') ubx_0 - upper bound on x for stage 0,',...
                    ' size [3]\n ');
sfun_input_names = [sfun_input_names; 'ubx_0 [3]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') parameters - concatenated for all stages 0 to N,',...
                    ' size [84]\n ');
sfun_input_names = [sfun_input_names; 'parameter_traj [84]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') y_ref_0 - size [2]\n ');
sfun_input_names = [sfun_input_names; 'y_ref_0 [2]'];
i_in = i_in + 1;



input_note = strcat(input_note, num2str(i_in), ') y_ref - concatenated for stages 1 to N-1,',...
                    ' size [57]\n ');
sfun_input_names = [sfun_input_names; 'y_ref [57]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') y_ref_e - size [2]\n ');
sfun_input_names = [sfun_input_names; 'y_ref_e [2]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') lh for stages 1 to N-1, size [19]\n ');
sfun_input_names = [sfun_input_names; 'lh [19]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') uh for stages 1 to N-1, size [19]\n ');
sfun_input_names = [sfun_input_names; 'uh [19]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') lh_0, size [1]\n ');
sfun_input_names = [sfun_input_names; 'lh_0 [1]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') uh_0, size [1]\n ');
sfun_input_names = [sfun_input_names; 'uh_0 [1]'];
i_in = i_in + 1;



input_note = strcat(input_note, num2str(i_in), ') cost_W_0 in column-major format, size [4]\n ');
sfun_input_names = [sfun_input_names; 'cost_W_0 [4]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') cost_W in column-major format, that is set for all intermediate stages: 1 to N-1, size [9]\n ');
sfun_input_names = [sfun_input_names; 'cost_W [9]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') cost_W_e in column-major format, size [4]\n ');
sfun_input_names = [sfun_input_names; 'cost_W_e [4]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') reset_solver - determines if iterate is set to all zeros before other initializations (x_init, u_init, pi_init) are set and before solver is called, size [1]\n ');
sfun_input_names = [sfun_input_names; 'reset_solver [1]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') x_init - initialization of x for all stages, size [63]\n ');
sfun_input_names = [sfun_input_names; 'x_init [63]'];
i_in = i_in + 1;

fprintf(input_note)

disp(' ')

output_note = 'Outputs are:\n';
i_out = 0;

global sfun_output_names
sfun_output_names = {};
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') u0, control input at node 0, size [1]\n ');
sfun_output_names = [sfun_output_names; 'u0 [1]'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') utraj, control input concatenated for nodes 0 to N-1, size [20]\n ');
sfun_output_names = [sfun_output_names; 'utraj [20]'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') xtraj, state concatenated for nodes 0 to N, size [63]\n ');
sfun_output_names = [sfun_output_names; 'xtraj [63]'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') acados solver status (0 = SUCCESS)\n ');
sfun_output_names = [sfun_output_names; 'solver_status'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') cost function value\n ');
sfun_output_names = [sfun_output_names; 'cost_value'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') KKT residuals, size [4] (stat, eq, ineq, comp)\n ');
sfun_output_names = [sfun_output_names; 'KKT_residuals [4]'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') x1, state at node 1\n ');
sfun_output_names = [sfun_output_names; 'x1 [3]'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') CPU time\n ');
sfun_output_names = [sfun_output_names; 'CPU_time'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') SQP iterations\n ');
sfun_output_names = [sfun_output_names; 'sqp_iter'];



fprintf(output_note)
modelName = 'MaglevAug_ocp_solver_simulink_block';
new_system(modelName);
open_system(modelName);

blockPath = [modelName '/MaglevAug_ocp_solver'];
add_block('simulink/User-Defined Functions/S-Function', blockPath);
set_param(blockPath, 'FunctionName', 'acados_solver_sfunction_MaglevAug');

Simulink.Mask.create(blockPath);

display_name = 'MaglevAug acados OCP';
input_labels = '';
for i = 1:length(sfun_input_names)
	input_labels = [input_labels, sprintf('port_label(''input'', %d, ''%s'')\n', i, sfun_input_names{i})];
end
output_labels = '';
for i = 1:length(sfun_output_names)
	output_labels = [output_labels, sprintf('port_label(''output'', %d, ''%s'')\n', i, sfun_output_names{i})];
end
mask_str = [input_labels, output_labels, sprintf('disp(''%s'')', display_name)];

mask = Simulink.Mask.get(blockPath);
mask.Display = mask_str;

save_system(modelName);
close_system(modelName);
disp([newline, 'Created the OCP solver Simulink block in: ', modelName])
