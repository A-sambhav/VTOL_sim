#include <ignition/gazebo/System.hh>

#include <ignition/gazebo/Model.hh>

#include <ignition/gazebo/Link.hh>

#include <ignition/gazebo/EntityComponentManager.hh>

#include <ignition/gazebo/components/JointVelocity.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/math/Vector3.hh>

#include <sdf/Element.hh>

#include <iostream>

namespace vtol
{

class MulticopterMotorModel
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{

public:

    // ======================================================
    //                    CONFIGURE
    // ======================================================

    void Configure(

        const ignition::gazebo::Entity &_entity,

        const std::shared_ptr<const sdf::Element> &_sdf,

        ignition::gazebo::EntityComponentManager &_ecm,

        ignition::gazebo::EventManager &

    ) override
    {

        // --------------------------------------------------
        // MODEL
        // --------------------------------------------------

        this->model = ignition::gazebo::Model(_entity);

        // --------------------------------------------------
        // READ PARAMETERS
        // --------------------------------------------------

        this->jointName =

            _sdf->Get<std::string>("joint_name");

        this->linkName =

            _sdf->Get<std::string>("link_name");

        this->motorConstant =

            _sdf->Get<double>("motor_constant");

        // --------------------------------------------------
        // GET JOINT ENTITY
        // --------------------------------------------------

        this->jointEntity =

            this->model.JointByName(

                _ecm,

                this->jointName

            );

        // --------------------------------------------------
        // GET LINK ENTITY
        // --------------------------------------------------

        this->linkEntity =

            this->model.LinkByName(

                _ecm,

                this->linkName

            );

        // --------------------------------------------------
        // DEBUG INFO
        // --------------------------------------------------

        std::cout << std::endl;

        std::cout
            << "===================================="
            << std::endl;

        std::cout
            << " Multicopter Motor Plugin Loaded "
            << std::endl;

        std::cout
            << " Joint : "
            << this->jointName
            << std::endl;

        std::cout
            << " Link  : "
            << this->linkName
            << std::endl;

        std::cout
            << "===================================="
            << std::endl;
    }

    // ======================================================
    //                    PRE UPDATE
    // ======================================================

    void PreUpdate(

        const ignition::gazebo::UpdateInfo &,

        ignition::gazebo::EntityComponentManager &_ecm

    ) override
    {

        // --------------------------------------------------
        // READ JOINT VELOCITY
        // --------------------------------------------------

        auto jointVelocity =

            _ecm.Component<
                ignition::gazebo::components::JointVelocity
            >(this->jointEntity);

        if (!jointVelocity)
            return;

        double omega =

            jointVelocity->Data()[0];

        // --------------------------------------------------
        // THRUST EQUATION
        //
        // F = k * omega^2
        // --------------------------------------------------

        double thrust =

            this->motorConstant *
            omega *
            omega;

        // --------------------------------------------------
        // APPLY FORCE
        // --------------------------------------------------

        ignition::gazebo::Link link(

            this->linkEntity

        );

        link.AddWorldForce(

            _ecm,

            ignition::math::Vector3d(

                0,
                0,
                thrust

            )

        );
    }

private:

    ignition::gazebo::Model model{

        ignition::gazebo::kNullEntity

    };

    ignition::gazebo::Entity jointEntity{

        ignition::gazebo::kNullEntity

    };

    ignition::gazebo::Entity linkEntity{

        ignition::gazebo::kNullEntity

    };

    std::string jointName;

    std::string linkName;

    double motorConstant{0.0005};
};

}

// ==========================================================
//                    REGISTER PLUGIN
// ==========================================================

IGNITION_ADD_PLUGIN(

    vtol::MulticopterMotorModel,

    ignition::gazebo::System,

    vtol::MulticopterMotorModel::ISystemConfigure,

    vtol::MulticopterMotorModel::ISystemPreUpdate

)