
Sandbox all the work
Create a fake robot class that just responsds to my auction manager stuff
make an auction manager that''s dumb, then extend it to work with skygrid


parser: parse incoming messages on both the robot side and the auction manager side
manager: manages auctions, keeps a running total of bids, and after timeout declares winners... 
robot: gets incoming messages, keeps a tab of all auctions, throws back bids



Definitions:
    Auction:
        Sends out a bundle (in this case a bundle can be of size 1 in which case we can call it an
        item) of points, the number of points, a timeout (0 if it simply waits for all robots),
        the type of auction.
        MESS: auc_id, auc_type, time_out, num_points, [point1, point2, ..., pointn]
        auc_id: Number
        auc_type: Number, defines for (RANDOM, SSI, OSI, SRC, PSI, etc)
        time_out: Number in miliseconds
        num_points: Number
        point: x, y

    Bid:
        A robot''s cost to complete an auction, plus the auc_id that it is bidding on, if two bids
        are received from the same robot then an error should probably be thrown (let''s assume for
        fun''s sake that we are under attack by someone malicious)
        MESS: rob_id, auc_id, bid
        rob_id: Number, robot''s id
        auc_id: Number, auction id
        bid: Number

Messages:
    AUCTION_START - start an auction, sent by the manager
    AUCTION_WON   - ends an auction, assigns points to robot, sent by the manager
    AUCTION_BID   - bid on an auction, sent by a robot

Hypothesis: Different allocation of target nodes to a team of robots will give
different results. Metrics: Minimax, Minisum, Miniave

Abstract:
Different allocation methods of target nodes to a team of robots will give
varying results of success when measured against three different measurements:
Minimax - minimize the maximum path cost of any single robot
Minisum - minimize the total path cost of all robots
Miniave - minimize the average of all path costs for all robots
Market based approaches
multi-robot coordination
task/resource allocation
