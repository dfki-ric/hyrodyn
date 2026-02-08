 /*********************************************************************
  * Software License Agreement (BSD License)
  *
  *  Copyright (c) 2013, Open Source Robotics Foundation
  *  All rights reserved.
  *
  *  Redistribution and use in source and binary forms, with or without
  *  modification, are permitted provided that the following conditions
  *  are met:
  *
  *   * Redistributions of source code must retain the above copyright
  *     notice, this list of conditions and the following disclaimer.
  *   * Redistributions in binary form must reproduce the above
  *     copyright notice, this list of conditions and the following
  *     disclaimer in the documentation and/or other materials provided
  *     with the distribution.
  *   * Neither the name of the Open Source Robotics Foundation
  *     nor the names of its contributors may be
  *     used to endorse or promote products derived
  *     from this software without specific prior written permission.
  *
  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  *  POSSIBILITY OF SUCH DAMAGE.
  *********************************************************************/
 
 #include <sstream>
 #include "transmission_parser.h"
 
 using namespace std;
 
 namespace transmission_interface
 {
 
 bool TransmissionParser::parse(const std::string& urdf, std::vector<TransmissionInfo>& transmissions)
 {
   // initialize TiXmlDocument doc with a string
   TiXmlDocument doc;
   
   if (!doc.Parse(urdf.c_str()) && doc.Error())
   {
     cerr<<"Can't parse transmissions. Invalid robot description."<<endl;
     return false;
   }
 
   // Find joints in transmission tags
   TiXmlElement *root = doc.RootElement();
 
   // Constructs the transmissions by parsing custom xml.
   TiXmlElement *trans_it = nullptr;
   for (trans_it = root->FirstChildElement("transmission"); trans_it;
        trans_it = trans_it->NextSiblingElement("transmission"))
   {
     transmission_interface::TransmissionInfo transmission;
 
     // Transmission name
     if(trans_it->Attribute("name"))
     {
       transmission.name_ = trans_it->Attribute("name");
       if (transmission.name_.empty())
       {
         cerr<<"URDF transmission parser: Empty name attribute specified for transmission."<<endl;
         continue;
       }
     }
     else
     {
       cerr<<"URDF transmission parser: No name attribute specified for transmission."<<endl;
       continue;
     }
 
     // Transmission type
     TiXmlElement *type_child = trans_it->FirstChildElement("type");
     if(!type_child)
     {
       cerr<<"URDF transmission parser: No type element found in transmission '"
         << transmission.name_ << "'."<<endl;
       continue;
     }
     if (!type_child->GetText())
     {
       cerr<<"URDF transmission parser: Skipping empty type element in transmission '"
                              << transmission.name_ << "'."<<endl;
       continue;
     }
     transmission.type_ = type_child->GetText();

     // Load custom transmission
     if(!parseCustomTransmission(trans_it, transmission.joints_))
     {
       cerr<<"URDF transmission parser: Failed to load custom transmission '"
         << transmission.name_ << "'."<<endl;
       continue;
     }
 
     // Save loaded transmission
     transmissions.push_back(transmission);
 
   } // end for <transmission>
 
   if( transmissions.empty() )
   {
     cerr<<"URDF transmission parser: No valid transmissions found."<<endl;
   }
 
   return true;
 }
  
 bool TransmissionParser::parseCustomTransmission(TiXmlElement *trans_it, std::vector<JointInfo>& joints)
 {
   // Loop through each available joint
   TiXmlElement *joint_it = nullptr;
   for (joint_it = trans_it->FirstChildElement("joint"); joint_it;
        joint_it = joint_it->NextSiblingElement("joint"))
   {
     // Create new joint
     transmission_interface::JointInfo joint;
 
     // Joint name
     if(joint_it->Attribute("name"))
     {
       joint.name_ = joint_it->Attribute("name");
       if (joint.name_.empty())
       {
         cerr<<"URDF transmission parser: Empty name attribute specified for joint."<<endl;
         continue;
       }
     }
     else
     {
       cerr<<"URDF transmission parser: No name attribute specified for joint."<<endl;
       return false;
     }
	 // cout<<"Joint found with name = "<<	joint.name_ << endl;
	 
     // Hardware interfaces (made optional in hyrodyn as this is a ROS feature!)
     TiXmlElement *jnt_dependency_it = nullptr;
     for (jnt_dependency_it = joint_it->FirstChildElement("depends_on"); jnt_dependency_it;
          jnt_dependency_it = jnt_dependency_it->NextSiblingElement("depends_on"))
     {

     // Depends on Joint name
     transmission_interface::DependentJointInfo dependent_joint;
     if(jnt_dependency_it->Attribute("joint"))
     {
       dependent_joint.name = jnt_dependency_it->Attribute("joint");
       if (dependent_joint.name.empty())
       {
         cerr<<"URDF transmission parser: Empty name attribute specified for the dependent joint."<<endl;
         continue;
       }
     }
     else
     {
       cerr<<"URDF transmission parser: No name attribute specified for the dependent joint."<<endl;
       return false;
     }
	 //cout<<"Dependent Joint found with name = "<< dependent_joint.name << endl;
	 
	// Get mimic multiplier
	const char* multiplier_str = jnt_dependency_it->Attribute("multiplier");

	if (multiplier_str == NULL)
	{
	cout << "URDF transmission parser: no multiplier, using default value of 1" << endl;
	dependent_joint.multiplier = 1;    
	}
	else
	{
	try {
	  dependent_joint.multiplier = atof(multiplier_str);
	} catch(std::runtime_error &) {
	  cerr<<"multiplier value (%s) is not a valid float" << multiplier_str << endl;
	  return false;
	}
	}
	//cout<<"Multiplier = "<< dependent_joint.multiplier << endl;  

	// Get mimic offset
	const char* offset_str = jnt_dependency_it->Attribute("offset");

	if (offset_str == NULL)
	{
	cout << "URDF transmission parser: no offset, using default value of 0" << endl;
	dependent_joint.offset = 0;    
	}
	else
	{
	try {
	  dependent_joint.offset = atof(offset_str);
	} catch(std::runtime_error &) {
	  cerr<<"offset value (%s) is not a valid float" << offset_str << endl;
	  return false;
	}
	}
	//cout<<"Offset = "<< dependent_joint.offset << endl;  
	joint.dependent_joints_.push_back(dependent_joint);	 
    }
    
   joints.push_back(joint); 
   }
   
   return true; 
 }
 } // namespace
