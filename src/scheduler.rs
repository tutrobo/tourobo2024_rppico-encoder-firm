use rp_pico::hal;

pub struct Scheduler<'a> {
    wait_tick: u64,
    next_update: u64,
    timer: &'a hal::Timer,
}

impl<'a> Scheduler<'a> {
    pub fn new(wait_tick: u64, timer: &'a hal::Timer) -> Scheduler<'a> {
        Scheduler::<'a> {
            wait_tick: wait_tick,
            next_update: timer.get_counter().ticks() + wait_tick,
            timer: timer,
        }
    }
    pub fn update(&mut self) -> bool {
        let tick = self.timer.get_counter().ticks();
        if tick >= self.next_update {
            self.next_update += self.wait_tick;
            return true;
        } else {
            return false;
        }
    }
}
