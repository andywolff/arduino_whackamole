#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Empty.h"

// Frequency of "turns"
// (smallest unit of time measurement in game)
// measured in hertz
#define TURN_RATE 10

// Number of turns in a single game
#define NUM_TURNS_IN_GAME 1200

// Number of turns a mole stays up if not whacked
#define MOLE_UP_TIME 100

// Probability that a given down mole will pop up on any given turn
#define POPUP_CHANCE 0.008

// Probability that a mole will be good, rather than evil
// 1.0 = always good
// 0.0 = always evil
#define MOLE_KARMA 0.3

// Number of turns a mole must stay down after going down
#define MIN_MOLE_DOWN_TIME 10

// Number of points awarded for hitting an evil mole
#define EVIL_POINTS 1

// Number of points deducted for hitting a good mole
#define GOOD_POINTS 3

// Indicates whether a game is in progress or not
static bool gameStarted;
// Number of turns left in the game
static int turnsLeft;

// Current Mole State described by array of ints. For each mole:
// 0 means that mole is down;
// Positive # means that the evil mole is up and has been up for that long;
// Negative # means that the evil mole is up and has been up for that long;
static int moleStates[7];

// For each mole:
// If mole is down, minimum # of remaining turns that it must remain down;
// If mole is up, always 0.
static int moleCooldown[7];

// Flag indicating that the mole state data has changed since last published.
static bool stateChanged;

// Keeps track of the user's score for the game
static int score;

// Flag indicating that the score has changed since last published
static bool scoreChanged;

// Flags indicating whether the robot base and arm, respectively, have moved
// to their starting positions
static bool robotBaseInit;
static bool robotArmInit;



// Initializes the game
void startGameCallback(const std_msgs::Int16 startValue);
// The game may only begin once the robot base has moved to its initial position
void robotPosCallback(const std_msgs::Int16::ConstPtr& msg);
// The game may only begin once the robot arm has moved to its initial position
void armPosCallback(const std_msgs::Int16::ConstPtr& msg);
// Puts down moles when they are whacked
void moleWhackCallback(const std_msgs::Int16::ConstPtr& msg);
// Publishes the mole states
void publishMoleStates(ros::Publisher pub);
// Publishes the player's score
void publishScore(ros::Publisher pub);
// Publishes the mole state information every turn for data logging purposes.
void publishStateData(ros::Publisher pub);

// Determines which moles should pop up and changes the mole states accordingly.
void checkDownMoles();
// Determines which moles have been up for the maximum time, and changes
// the mole states so that they go down. Increments the time the others have been up.
void checkUpMoles();

// Sets initial mole Positions
void initializeMoleStates();

/*
 * Node controlling whack-a-mole game logic
 */
int main(int argc, char **argv)
{
  // Initialize node
  ros::init(argc, argv, "whackamole_game_node");
  ros::NodeHandle n;

  // Subscriber listening for command to start game
  ros::Subscriber startGameSub = n.subscribe("whackamole/startgame", 1000, startGameCallback);

  // Subscriber listening for when moles are whacked
  ros::Subscriber moleWhackSub = n.subscribe("whackamole/whack_impact", 1000, moleWhackCallback);

  // Subscriber listening for response from robot upon reaching initial position
  ros::Subscriber robotPosSub = n.subscribe("whackamole/robot_position_arrive", 1000, robotPosCallback);

  // Subscriber listening for response from robot arm upon reaching initial orientation
  ros::Subscriber armPosSub = n.subscribe("whackamole/whack_complete", 1000, armPosCallback);

  // Publisher publishing mole states
  ros::Publisher moleStatePub = n.advertise<std_msgs::Int32MultiArray>("whackamole/mole_states", 100);

  // Publisher publishing state data for data logging
  ros::Publisher stateDataPub = n.advertise<std_msgs::Int32MultiArray>("whackamole/state_data", 100);

  // Publisher publishing score updates
  ros::Publisher scorePub = n.advertise<std_msgs::Int16>("whackamole/score", 100);

  // Publisher publishing messenges indicating time remaining in the game
  ros::Publisher timeLeftPub = n.advertise<std_msgs::Int16>("whackamole/time_left", 100);

  ros::Publisher robotInitPub = n.advertise<std_msgs::Empty>("whackamole/robot_init", 100);

  // Publisher that publishes a message when the game actually starts, after initializing.
  ros::Publisher gameStartedPub = n.advertise<std_msgs::Empty>("whackamole/game_started", 100);

  // Set loop rate
  ros::Rate loop_rate(TURN_RATE);


  // set random seed for game
  unsigned int seed = time(NULL);
  srand(seed);

  // Game initially has not started
  gameStarted = 0;
  // Assume no initial state change
  stateChanged = 0;

  // Assume robot is not in initial position until told otherwise
  robotBaseInit = 0;
  robotArmInit = 0;
  bool sentRobotInitCmd = 0;

  while (ros::ok())
  {
    if(gameStarted && robotBaseInit && robotArmInit) {
      // If the game has just started, send a game_started message      
      std_msgs::Empty gameStartedMessage;
      gameStartedPub.publish(gameStartedMessage);

      // If the mole states have changed, publish the new states.
      if(stateChanged) {
        ROS_INFO("State changed");
        publishMoleStates(moleStatePub);
        stateChanged = 0;
      }
      // If the player's score has changed, publish the new score.
      if(scoreChanged) {
        ROS_INFO("Score changed");
        publishScore(scorePub);
        scoreChanged = 0;
      }
      // Publish current state data for data logging purposes
      publishStateData(stateDataPub);
      // Determine which moles should pop up and change the mole states accordingly.
      checkDownMoles();
      // Determine which moles have been up for the maximum time, and change
      // the mole states so that they go down. Increment the time the others have been up.
      checkUpMoles();
      // Decrement num turns left
      turnsLeft--;

      // If at least another full second has ticked off the clock
      if(TURN_RATE < 1 || ((turnsLeft % TURN_RATE) == 0)) {
        // Publish time remaining in the game
        std_msgs::Int16 timeLeftMessage;
        // Compute Time left
        timeLeftMessage.data = (int) (turnsLeft / TURN_RATE);
        // Publish the message
        timeLeftPub.publish(timeLeftMessage);
      }

      // Check for game end
      if(turnsLeft <= 0) {
        gameStarted = 0;
        // Assume robot is not in initial position until told otherwise
        robotBaseInit = 0;
        robotArmInit = 0;
        sentRobotInitCmd = 0;
        // Put all moles back down
        initializeMoleStates();
        publishMoleStates(moleStatePub);
        ROS_INFO("End of Game");
      }
    } else if(gameStarted && !sentRobotInitCmd) {
      // If the game has been started, send an init command to the robot
      // This tells the robot to move to the center position.
      // The game will not begin until it hears back that the robot
      // has reached the position successfully
      std_msgs::Empty robotInitMessage;
      robotInitPub.publish(robotInitMessage);
      sentRobotInitCmd = 1;
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

}

// Determines which moles should pop up and changes the mole states accordingly.
void checkDownMoles()
{
  // For each mole
  int i;
  for(i=0; i<7; i++) {
    // Only deal with moles that are down
    if(moleStates[i] == 0) {
      // If not in cooldown, there is a chance it will pop up.
      if(moleCooldown[i] <= 0) {
        // Determine whether mole comes up
        double r = ((double) rand() / (RAND_MAX));
        if(r < POPUP_CHANCE) {
          // Determine whether good or evil mole pops up
          double randKarma = ((double) rand() / (RAND_MAX));
          if(randKarma < MOLE_KARMA) {
            moleStates[i] = 1;
          } else {
            moleStates[i] = -1;
          }
          // Mole states have changed
          stateChanged = 1;
        }
      } else {
        // Otherwise, decrement the cooldown time
        moleCooldown[i]--;
      }
    }
  }
}

// Determines which moles have been up for the maximum time, and changes
// the mole states so that they go down. Increments the time the others have been up.
void checkUpMoles()
{
  // For each mole
  int i;
  for(i=0; i<7; i++) {
    // Only deal with moles that are up
    if(moleStates[i] != 0) {
      // If mole has been up for maximum time, put mole down
      if(moleStates[i] >= MOLE_UP_TIME || moleStates[i] <= (-1 * MOLE_UP_TIME)) {
        moleStates[i] = 0;
        moleCooldown[i] = MIN_MOLE_DOWN_TIME;
        // Mole states have changed
        stateChanged = 1;
      } else {
        // Otherwise increment the time the mole has been up
        if(moleStates[i] > 0) {
          moleStates[i]++;
        } else {
          moleStates[i]--;
        }
      }
    }
  }
}

// Publishes the mole states
void publishMoleStates(ros::Publisher pub)
{
  // Create array to hold state data
  std_msgs::Int32MultiArray stateArray;
  stateArray.data.clear();
  // Put moleState data into the array
  int i;
  for(i=0; i<7; i++) {
    stateArray.data.push_back(moleStates[i]);
  }
  // Publish the array
  pub.publish(stateArray);
}

// Publishes the mole state information every turn for data logging purposes.
void publishStateData(ros::Publisher pub)
{
  // Create array to hold state data
  std_msgs::Int32MultiArray stateArray;
  stateArray.data.clear();
  // Put moleState data into the array
  int i;
  for(i=0; i<7; i++) {
    stateArray.data.push_back(moleStates[i]);
  }
  // Publish the array
  pub.publish(stateArray);
}

// Publishes the player's score
void publishScore(ros::Publisher pub)
{
  // Create message to hold score data
  std_msgs::Int16 scoreMessage;
  // Put the score into the message
  scoreMessage.data = score;
  // Publish the message
  pub.publish(scoreMessage);
}

// Sets initial mole Positions
void initializeMoleStates()
{
  int i;
  // Set all moles to down
  for(i = 0; i < 7; i++) {
    moleStates[i] = 0;
    moleCooldown[i] = 0;
  }
  // The state of the moles has changed
  stateChanged = 1;
}

// Starts the game
void startGameCallback(const std_msgs::Int16)
{
  if(!gameStarted) {
    // Set total number of turns for game to run
    turnsLeft = NUM_TURNS_IN_GAME + 1;
    // Initializes mole states (puts them all down)
    initializeMoleStates();
    // Randomly choose one evil mole to raise at the start of the game.
    int startMole = rand() % 7;
    moleStates[startMole] = -1;
    // Sets the player score to zero
    score = 0;
    // Starts the game
    gameStarted = 1;
  }
}

// Puts down moles when they are whacked
void moleWhackCallback(const std_msgs::Int16::ConstPtr& msg)
{
  int whackedMole = msg->data;
  // If the mole was hit
  if(whackedMole >= 0 && whackedMole < 7 && moleStates[whackedMole] != 0) {
    // Check whether mole hit was good or evil, and adjust score accordingly
    if(moleStates[whackedMole] < 0) {
      score += EVIL_POINTS;
    } else if(moleStates[whackedMole] > 0) {
      score -= GOOD_POINTS;
    }
    scoreChanged = 1;

    // Turn down mole that was whacked and begin cooldown
    moleStates[whackedMole] = 0;
    moleCooldown[whackedMole] = 0;
    // The mole state has changed
    stateChanged = 1;
  }
}

// The game may only begin once the robot base has moved to its initial position
void robotPosCallback(const std_msgs::Int16::ConstPtr& msg)
{
  robotBaseInit = 1;
}

// The game may only begin once the robot arm has moved to its initial position
void armPosCallback(const std_msgs::Int16::ConstPtr& msg)
{
  robotArmInit = 1;
}
