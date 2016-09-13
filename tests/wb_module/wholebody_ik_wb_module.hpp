/* Copyright [2016] [Alessandro Settimi (ale.settimi@gmail.com), Mirko Ferrati, Danilo Caporale, Edoardo Farnioli]
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

#ifndef WHOLEBODY_IK_WB_MODULE_HPP_
#define WHOLEBODY_IK_WB_MODULE_HPP_

#include <GYM/control_module.hpp>

#include "wholebody_ik_wb_module_thread.h"

/**
 * @brief wholebody_ik module derived from control_module
 * 
 * @author 
 */

using namespace walkman;

class wholebody_ik_wb_module : public control_module<wholebody_ik_wb_thread> {
public:
    
    /**
     * @brief constructor: do nothing but construct the superclass
     * 
     */
    wholebody_ik_wb_module(    int argc, 
                               char* argv[],
                               std::string module_prefix, 
                               int module_period, 
                               yarp::os::ResourceFinder rf ) : control_module<wholebody_ik_wb_thread>(  argc, 
                                                                                                        argv, 
                                                                                                        module_prefix, 
                                                                                                        module_period,
                                                                                                        rf )
    {
    }
    
    /**
     * @brief overriden function to specify the custom params for the param helper
     * 
     * @return a vector of the custom params for the param helper
     */
    virtual std::vector< paramHelp::ParamProxyInterface* > custom_get_ph_parameters() 
    {
        // TODO: function skeleton
        std::vector<paramHelp::ParamProxyInterface *> custom_params;
        return custom_params;
    }
    
    
};

#endif //WHOLEBODY_IK_WB_MODULE_HPP_
