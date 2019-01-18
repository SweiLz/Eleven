#! /usr/bin/env python

import rospy
import smach
import smach_ros


class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome2'],
                             input_keys=['foo_counter_in'],
                             output_keys=['foo_counter_out'])

    def execute(self, userdata):
        rospy.loginfo("Exe FOO")
        if int(userdata.foo_counter_in) < 3:
            userdata.foo_counter_out = userdata.foo_counter_in
            return 'outcome1'
        else:
            return 'outcome2'


class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome2'],
                             input_keys=['bar_counter_in'])

    def execute(self, userdata):
        rospy.loginfo("Exe Bar")
        rospy.loginfo("Counter = %s" % userdata.bar_counter_in+"555")
        return 'outcome2'


if __name__ == "__main__":
    rospy.init_node("smach_example_state_machine")
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])
    sm.userdata.sm_counter = "2"
    with sm:
        smach.StateMachine.add('FOO', Foo(),
                               transitions={'outcome1': 'BAR',
                                            'outcome2': 'outcome4'},
                               remapping={'foo_counter_in': 'sm_counter',
                                          'foo_counter_out': 'sm_counter'})

        smach.StateMachine.add('BAR', Bar(), transitions={'outcome1': 'FOO', 'outcome2': 'outcome5'},
                               remapping={'bar_counter_in': 'sm_counter'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
