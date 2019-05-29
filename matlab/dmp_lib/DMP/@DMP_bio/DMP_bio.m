%% DMP_bio class
%

classdef DMP_bio < DMP_
       
    methods  (Access = public)
        %% DMP constructor.
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_clock_ptr: Pointer to a DMP canonical system object.
        function this = DMP_bio(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr)
            
            if (nargin < 4), can_clock_ptr = CanonicalClock(); end
            if (nargin < 5), shape_attr_gating_ptr=SigmoidGatingFunction(1.0, 0.5); end
            this@DMP_(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr);

        end
        
        
        function Fd = calcFd(this, x, y, dy, ddy, y0, g)

            s_attr_gating = this.shapeAttrGating(x);
            tau = this.getTau();
            K = this.a_z * this.b_z;
            Fd = (ddy*tau^2 - this.goalAttractor(x, y, tau*dy, g) + K*(g-y0)*s_attr_gating);

        end
        
        
        function Fd = calcLearnedFd(this, x, y0, g)
            
            s = this.shapeAttrGating(x) * this.forcingTermScaling(y0, g);
            Fd = this.shapeAttractor(x, y0, g) + s*(g-y0);

        end
        
        
        function f_scale = forcingTermScaling(this, y0, g)

            f_scale = this.a_z*this.b_z;

        end
        
        
        function shape_attr = shapeAttractor(this, x, y0, g)
            
            s_attr_gating = this.shapeAttrGating(x);
            f_scale = this.forcingTermScaling(y0, g);
            shape_attr = s_attr_gating * f_scale * (this.forcingTerm(x) - (g-y0));

        end


    end
    

end
