/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class AnimatedBox : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      std::string model_name = this->model->GetName();
      std::cout << "LOADING MODELPlugin FOR model " << model_name << std::endl;

      double rtt = 3.0;
        // create the animation
        gazebo::common::PoseAnimationPtr anim(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test", rtt, true));
        gazebo::common::PoseKeyFrame *key;

        double roomlen = 3.0;

        if (model_name.substr(0,1).compare("1") == 0)
        {
          // set starting location of the box
          key = anim->CreateKeyFrame(0);
          key->Translation(ignition::math::Vector3d(-0.75, -1.5, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));

          // set waypoint location after 2 seconds
          key = anim->CreateKeyFrame(rtt/2.0);
          key->Translation(ignition::math::Vector3d(0.75, -1.5, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));


          key = anim->CreateKeyFrame(rtt);
          key->Translation(ignition::math::Vector3d(-0.75, -1.5, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));
          
        }

        else if (model_name.substr(0,1).compare("2") == 0)
        {
          // set starting location of the box
          key = anim->CreateKeyFrame(0);
          key->Translation(ignition::math::Vector3d(1.0, 2.25, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));

          // set waypoint location after 2 seconds
          key = anim->CreateKeyFrame(rtt/2.0);
          key->Translation(ignition::math::Vector3d(1.0, 3.75, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));


          key = anim->CreateKeyFrame(rtt);
          key->Translation(ignition::math::Vector3d(1.0, 2.25, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));
        }

        else if (model_name.substr(0,1).compare("3") == 0)
        {
          // set starting location of the box
          key = anim->CreateKeyFrame(0);
          key->Translation(ignition::math::Vector3d(-6.4, 2.25, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));

          // set waypoint location after 2 seconds
          key = anim->CreateKeyFrame(rtt/2.0);
          key->Translation(ignition::math::Vector3d(-6.4, 3.75, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));


          key = anim->CreateKeyFrame(rtt);
          key->Translation(ignition::math::Vector3d(-6.4, 2.25, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));
        }

        else if (model_name.substr(0,1).compare("4") == 0)
        {
          // set starting location of the box
          key = anim->CreateKeyFrame(0);
          key->Translation(ignition::math::Vector3d(-5.5, -0.5, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));

          // set waypoint location after 2 seconds
          key = anim->CreateKeyFrame(rtt/2.0);
          key->Translation(ignition::math::Vector3d(-7, -0.5, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));


          key = anim->CreateKeyFrame(rtt);
          key->Translation(ignition::math::Vector3d(-5.5, -0.5, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));
        }

        else if (model_name.substr(0,1).compare("5") == 0)
        {
          // set starting location of the box
          key = anim->CreateKeyFrame(0);
          key->Translation(ignition::math::Vector3d(5.0, 2.5, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));

          // set waypoint location after 2 seconds
          key = anim->CreateKeyFrame(rtt/2.0);
          key->Translation(ignition::math::Vector3d(5, 4.0, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));


          key = anim->CreateKeyFrame(rtt);
          key->Translation(ignition::math::Vector3d(5.0, 2.5, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));
        }

        else if (model_name.substr(0,1).compare("6") == 0)
        {
          // set starting location of the box
          key = anim->CreateKeyFrame(0);
          key->Translation(ignition::math::Vector3d(6.5, -4.5, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));

          // set waypoint location after 2 seconds
          key = anim->CreateKeyFrame(rtt/2.0);
          key->Translation(ignition::math::Vector3d(6.5, -3.0, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));


          key = anim->CreateKeyFrame(rtt);
          key->Translation(ignition::math::Vector3d(6.5, -4.5, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));
        }

        else if (model_name.substr(0,1).compare("7") == 0)
        {
          // set starting location of the box
          key = anim->CreateKeyFrame(0);
          key->Translation(ignition::math::Vector3d(-1.5, 2.5, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));

          // set waypoint location after 2 seconds
          key = anim->CreateKeyFrame(rtt/2.0);
          key->Translation(ignition::math::Vector3d(-3.0, 2.5, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));


          key = anim->CreateKeyFrame(rtt);
          key->Translation(ignition::math::Vector3d(-1.5, 2.5, 0.5));
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));
        }


        // key = anim->CreateKeyFrame(6.0);
        // key->Translation(ignition::math::Vector3d(-10, 20, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(8.0);
        // key->Translation(ignition::math::Vector3d(10, -20, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));

        // // set final location equal to starting location
        // key = anim->CreateKeyFrame(10);
        // key->Translation(ignition::math::Vector3d(0, 0, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set the animation
        _parent->SetAnimation(anim);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
}
