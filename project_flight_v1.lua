--
-- project code by Roman Novokshanov
--
local SCRIPT_NAME     = 'project'

local RANDOM_SEED = 42 -- set fixed random seed to reproducibility otherwise use os.time()
local SIGNAL_RECEIVED_THRESHOLD = 0.8 -- threshold for simulating signal received event (0 to 1)
local NEW_WP_RECEIVED_THRESHOLD = 0.8 -- threshold for simulating new wp received event (0 to 1)
local RUN_INTERVAL_MS = 2000 -- loop frequency

local DELTA_LAT = 0.01 * 1e7 -- degrees * 1e7: 0.001 * 1e7 ~ 100m
local DELTA_LNG = - 0.005 * 1e7
local TARGET_REACHED_RADIUS = 100 -- 50 m

local NUM_CIRCLES = 20 -- 20 sec for circles manoeuvre
local NUM_GAIN_ALT = 10 -- 5 sec for gaining altitude manoeuvre

local TOTAL_CNT_SIM_GAIN_ALT = 2 -- max number gain alt events simulated
local TOTAL_CNT_SIM_NEW_WP = 1 -- max number new wp events simulated
local current_cnt_sim_gain_alt = 0 -- counter for gain alt events simulated
local current_cnt_sim_new_wp = 0 -- counter for new wp events simulated

local flag_wp_reached = false -- flag for checking if wp is reached
local flag_signal_received = false -- flag for checking if signal is recived
local flag_new_wp_received = false -- flag for event new wp received

local circle_count = 0 -- circles counter
local gain_alt_count = 0 -- gain altitude time counter
local current_plane_state = nil
local new_target = nil

local RCMAP_ROLL = rc:get_channel(1)
local RCMAP_PITCH = rc:get_channel(2)
local RCMAP_THROTTLE = rc:get_channel(3)
local RCMAP_YAW = rc:get_channel(4)
local RC5_OPTION = rc:get_channel(5)
local RC6_OPTION = rc:get_channel(6)

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local PLANE_MODE = {PLANE_MODE_CIRCLE=1, PLANE_MODE_AUTO=10, PLANE_MODE_RTL=11, PLANE_MODE_LOITER=12, PLANE_MODE_GUIDED=15, PLANE_MODE_FLY_BY_WIRE_A = 5}
local PLANE_STATE = {PLANE_STATE_CIRCLE=1, PLANE_STATE_AUTO=10, PLANE_STATE_RTL=11, PLANE_STATE_LOITER=12, PLANE_STATE_GUIDED=15, PLANE_STATE_GAIN_ALT=101, PLANE_STATE_NEW_WP=102}

local prev_nav_index = 0 -- previous wp index

-- manoeuvre params
local ROLL_VALUE = 1800
local PITCH_VALUE = 2000
local THROTTLE_VALUE = 2100
local YAW_VALUE = 1800


--------------------------------------------------------------
--- auxiliary functions
--------------------------------------------------------------

-- manoeuvre: rapid gain altitute and turn to side
local function do_gain_altitute(roll, pitch, throttle, yaw)
    RCMAP_ROLL:set_override(roll)
    RCMAP_PITCH:set_override(pitch)
    RCMAP_THROTTLE:set_override(throttle)
    RCMAP_YAW:set_override(yaw)
end

local function set_current_plane_state(state)
    current_plane_state = state
end

local function get_current_plane_state()
    return current_plane_state
end

local function gcs_msg(severity, txt)
    gcs:send_text(severity, string.format('%s: %s', SCRIPT_NAME, txt))
end

local function wp_reached()
    return flag_wp_reached
end

local function set_wp_reached(value)
    flag_wp_reached = value
end

local function new_wp_received()
    return flag_new_wp_received
end

local function set_new_wp_received(value)
    flag_new_wp_received = value
end

local function signal_received()
    return flag_signal_received
end

local function set_signal_received(value)
    flag_signal_received = value
end

-- monte carlo random signal generation
local function simulate_signal_received()

    -- generate randomly signal for gain alt maneuver
    if not signal_received() and not new_wp_received() then
        dist = vehicle:get_wp_distance_m()        
        if current_cnt_sim_gain_alt < TOTAL_CNT_SIM_GAIN_ALT and 1600 > dist and dist > 1000 then
            rnd = math.random()            
            if rnd > SIGNAL_RECEIVED_THRESHOLD then
                gcs_msg(MAV_SEVERITY.INFO, 'signal gain alt rnd: '..tostring(rnd))
                set_signal_received(true)
                current_cnt_sim_gain_alt = current_cnt_sim_gain_alt + 1
            end
        end
    end

    -- generate randomly signal for adding new wp
    if not signal_received() and not new_wp_received() then
        dist = vehicle:get_wp_distance_m()        
        if current_cnt_sim_new_wp < TOTAL_CNT_SIM_NEW_WP and 1800 > dist and dist > 1000 then
            rnd = math.random()            
            if rnd > NEW_WP_RECEIVED_THRESHOLD then
                gcs_msg(MAV_SEVERITY.INFO, 'signal new wp rnd: '..tostring(rnd))
                set_new_wp_received(true)
                current_cnt_sim_new_wp = current_cnt_sim_new_wp + 1
            end
        end
    end
end

function get_new_location(cur_loc, delta_lat, delta_lng)
    local new_loc = Location()
    
    
    new_loc:lat(cur_loc:lat() + delta_lat)
    new_loc:lng(cur_loc:lng() + delta_lng)
    
    new_loc:relative_alt(false)
    new_loc:terrain_alt(false)
    new_loc:origin_alt(false)
    new_loc:change_alt_frame(0)
    new_loc:alt(math.floor(cur_loc:alt()))
    return new_loc
 end


----------------------------------------------------------------
-- states functons
----------------------------------------------------------------
function state_auto()
    if wp_reached() then return update, RUN_INTERVAL_MS end
    if signal_received() then return update, RUN_INTERVAL_MS end
    if new_wp_received() then return update, RUN_INTERVAL_MS end

    current_mode = vehicle:get_mode()
    if current_mode ~= PLANE_MODE.PLANE_MODE_AUTO then
        vehicle:set_mode(PLANE_MODE.PLANE_MODE_AUTO)
        gcs_msg(MAV_SEVERITY.INFO, 'STATE AUTO STARTED')
        set_current_plane_state(PLANE_STATE.PLANE_STATE_AUTO)
    end

    -- condition for exiting current state AUTO: wp is reached
    current_nav_index = math.max(mission:get_current_nav_index(), 1)
    if current_nav_index <= 2 then return state_auto, RUN_INTERVAL_MS end
    if prev_nav_index < current_nav_index then
        prev_nav_index = current_nav_index
        set_wp_reached(true)
    end

    -- do monte carlo to generate random signal received events (only in AUTO mode)
    current_plane_state = get_current_plane_state()
    if current_plane_state == PLANE_STATE.PLANE_STATE_AUTO then
        simulate_signal_received()
    end

    return state_auto, RUN_INTERVAL_MS
end

-- state functions
function state_circle()
    if not wp_reached() then return update, RUN_INTERVAL_MS end

    circle_count = circle_count + 1
    current_mode = vehicle:get_mode()
    if current_mode ~= PLANE_MODE.PLANE_MODE_CIRCLE then
        vehicle:set_mode(PLANE_MODE.PLANE_MODE_CIRCLE)
        gcs_msg(MAV_SEVERITY.ALERT, 'WP REACHED. WAITING..')
        gcs_msg(MAV_SEVERITY.INFO, 'STATE CIRCLE STARTED')
    end

    -- condition for exiting current state CIRCLE: completed NUM_CIRCLES time counter
    if circle_count > NUM_CIRCLES then
        set_wp_reached(false)
        circle_count = 0
    end
    if circle_count > 0 then
        gcs_msg(MAV_SEVERITY.INFO, 'WAITING..'..tostring(circle_count))
    end
    return state_circle, RUN_INTERVAL_MS
end

-- state gain alt
function state_gain_alt()
    if not signal_received() then return update, RUN_INTERVAL_MS end

    gain_alt_count = gain_alt_count + 1
    current_plane_state = get_current_plane_state()
    current_mode = vehicle:get_mode()
    if current_mode == PLANE_MODE.PLANE_MODE_AUTO and current_plane_state ~= PLANE_STATE.PLANE_STATE_GAIN_ALT then
        vehicle:set_mode(PLANE_MODE.PLANE_MODE_FLY_BY_WIRE_A)
        set_current_plane_state(PLANE_STATE.PLANE_STATE_GAIN_ALT)
        gcs_msg(MAV_SEVERITY.ALERT, 'SIGNAL RECEIVED')
        gcs_msg(MAV_SEVERITY.INFO, 'STATE GAIN ALT STARTED')
        -- increase altitute: roll, pitch, throttle, yaw        
        do_gain_altitute(ROLL_VALUE, PITCH_VALUE, THROTTLE_VALUE, YAW_VALUE)
    end

    -- condition for exiting current state GAIN ALT: completed NUM_GAIN_ALT time counter
    if current_plane_state == PLANE_STATE.PLANE_STATE_GAIN_ALT and gain_alt_count > NUM_GAIN_ALT then
        set_signal_received(false)
        gain_alt_count = 0
        gcs_msg(MAV_SEVERITY.INFO, 'STATE GAIN ALT COMPLETED')
    end
    if gain_alt_count > 0 then
        gcs_msg(MAV_SEVERITY.INFO, 'GAINING ALT..'..tostring(gain_alt_count))
    end

    return state_gain_alt, RUN_INTERVAL_MS
end

-- state new wp
function state_new_wp()
    if not new_wp_received() then return update, RUN_INTERVAL_MS end

    current_plane_state = get_current_plane_state()
    current_mode = vehicle:get_mode()

    -- add new wp to the flight plan
    if current_mode == PLANE_MODE.PLANE_MODE_AUTO and current_plane_state ~= PLANE_STATE.PLANE_STATE_NEW_WP then
        vehicle:set_mode(PLANE_MODE.PLANE_MODE_GUIDED)
        set_current_plane_state(PLANE_STATE.PLANE_STATE_NEW_WP)
        gcs_msg(MAV_SEVERITY.ALERT, 'NEW WP RECEIVED')
        gcs_msg(MAV_SEVERITY.INFO, 'STATE ADD NEW WP STARTED')
        -- add wp
        new_target = get_new_location(ahrs:get_location(), DELTA_LAT, DELTA_LNG)
        vehicle:update_target_location(vehicle:get_target_location(), new_target)
        
        current_location = ahrs:get_position()
        dist_to_target = current_location:get_distance(new_target)
        gcs_msg(MAV_SEVERITY.INFO, 'dist to target loc: '..tostring(dist_to_target))
    end

    -- condition for compliting new wp state
    current_location = ahrs:get_position()
    dist_to_target = current_location:get_distance(new_target)
    if current_plane_state == PLANE_STATE.PLANE_STATE_NEW_WP and vehicle:get_mode() == PLANE_MODE.PLANE_MODE_GUIDED and dist_to_target < TARGET_REACHED_RADIUS then
        set_new_wp_received(false)
        set_current_plane_state(PLANE_STATE.PLANE_STATE_AUTO)
        set_wp_reached(true)
        gcs_msg(MAV_SEVERITY.INFO, 'STATE NEW WP COMPLETED')
    end

    return state_new_wp, RUN_INTERVAL_MS
end

function update()

    if not arming:is_armed() then return update, RUN_INTERVAL_MS end
    if not ahrs:healthy() then return update, RUN_INTERVAL_MS end

    if wp_reached() then
        -- circle mode        
        return state_circle, RUN_INTERVAL_MS
    end

    if signal_received() then
        -- gain alt mode        
        return state_gain_alt, RUN_INTERVAL_MS
    end

    if new_wp_received() then
        -- add new wp and move towards it   
        return state_new_wp, RUN_INTERVAL_MS
    end

    -- auto mode
    return state_auto, RUN_INTERVAL_MS
end

math.randomseed(RANDOM_SEED)
set_new_wp_received(false)
set_wp_reached(false)
set_signal_received(false)
set_current_plane_state(vehicle:get_mode())
gcs_msg(MAV_SEVERITY.INFO, 'Project flight initialized.')

return update()
