from H3DInterface import *

# Get references from X3D file
collisionSensor, sound= references.getValue()

# Some constant parameters for sound generation
speedToIntensity= 5             # Multiply speed of collision by this number to get sound intensity
minSeparation= 0.01             # To avoid a loud sound being masked by a related quieter sound, always leave
                                # this length of time (secs) between sounds

class ContactDetails:
    """ Information stored about each contact from the previous frame"""
    def __init__( self, repeatTime, position ):
        self.repeatTime= repeatTime
        self.position= position

class UpdateSound ( AutoUpdate ( MFNode ) ):
    """ A field to which a CollisionSensor's contacts field can be routed in order to generate sounds for collisions"""
    def __init__ ( self ):
        AutoUpdate ( MFNode ).__init__( self )
        self.prevContacts= {}       # A dictionary of information about contacts in the previous frame
                                    # indexed by the bodies involved
        self.lastSoundTime= 0       # The time at which the last sound was played
    
    def update ( self, event ):
        """ Check if there are any contacts between bodies, which were not present in the previous frame.
            If so, play a sound to represent the collision."""

        # A dictionary of information about contacts in 'current' frame
        curContacts= {}

        # For each contact in the 'current' frame
        for c in event.getValue():
            key= str(c.body1.getValue())+str(c.body2.getValue())    # A string to index the contact

            # if the contact is new this frame
            if key not in self.prevContacts:
                self.playSound ( c )    # play collision sound
                curContacts[key]= ContactDetails(time.getValue()+minSeparation,c.position.getValue())    # Store info about collision
            else:
                # Contact is not new, but if position has changed, it is a candidate for a collision sound
                # If position has not changed since last frame it is likely not relevant (maybe due to bug in CollisionSensor?)
                if self.prevContacts[key].repeatTime<time.getValue() and self.prevContacts[key].position!=c.position.getValue():
                    self.playSound ( c )
                    curContacts[key]= ContactDetails(time.getValue()+minSeparation,c.position.getValue())
                else:
                    curContacts[key]= ContactDetails(self.prevContacts[key].repeatTime,c.position.getValue())

        self.prevContacts= curContacts  # Store current contacts for comparison next frame

        return event.getValue()

    def playSound ( self, contact ):
        """ Play a sound to represent the collision given by the specified contact. The intensity of the sound
            is based on the velocity information obtained from the contact"""

        # Check that the minimum sound separation time has passed
        if time.getValue()-self.lastSoundTime < minSeparation:
            return

        contactNormal= contact.contactNormal.getValue()
        body1= contact.body1.getValue()
        body2= contact.body2.getValue()

        # Project the body 1's velocity onto the normal of the contact
        if body1:
            v1= contactNormal.dotProduct ( body1.linearVelocity.getValue() ) * contactNormal
        else:
            v1= Vec3f()

        # Project the body 2's velocity onto the normal of the contact
        if body2:
            v2= contactNormal.dotProduct ( body2.linearVelocity.getValue() ) * contactNormal
        else:
            v2= Vec3f()

        # Calculate the combined speed of both bodies along the normal of the contact
        speed= (v1-v2).length()

        # If the speed is over a certain threshold, play a sound for the collision
        if speed > 0.01:

            # Calculate the intensity of the sound based on the speed
            intensity= speed*speedToIntensity
            if intensity>1:
                intensity= 1

            # Play the sound
            sound.intensity.setValue ( intensity )
            sound.source.getValue().startTime.setValue ( time.getValue() )
            sound.source.getValue().stopTime.setValue ( time.getValue()+sound.source.getValue().duration_changed.getValue() )

            self.lastSoundTime= time.getValue()

# The field to which the CollisionSensor's contacts are routed to generate collision sounds
updateSound= UpdateSound()

collisionSensor.contacts.route ( updateSound )  # Set up the route to the UpdateSound field
