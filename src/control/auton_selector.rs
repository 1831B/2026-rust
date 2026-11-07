#[derive(Clone, Copy)]
pub struct RoutineStart {
    pub x: f64,
    pub y: f64,
    pub h: f64,
}

pub struct AutonomousRoutine {
    pub name: String,
    pub description: String,
    pub start: Option<RoutineStart>,
}

pub struct AutonomousSelector {
    pub routines: std::vec::Vec<AutonomousRoutine>,
    pub selection: usize,
    pub confirmed: bool,
}

impl Default for AutonomousSelector {
    fn default() -> Self {
        Self::new()
    }
}

impl AutonomousSelector {
    pub fn new() -> Self {
        Self {
            routines: std::vec::Vec::new(),
            selection: 0,
            confirmed: false,
        }
    }

    pub fn add_routine(&mut self, name: &str, desc: &str, start: Option<RoutineStart>) {
        self.routines.push(AutonomousRoutine {
            name: name.to_string(),
            description: desc.to_string(),
            start,
        });
    }

    pub fn start_pose(&self) -> Option<RoutineStart> {
        self.routines.get(self.selection).and_then(|r| r.start)
    }

    pub fn select(&mut self, index: isize) {
        if self.routines.is_empty() {
            return;
        }
        let n = self.routines.len() as isize;
        self.selection = (((index % n) + n) % n) as usize;
        self.confirmed = false;
    }

    pub fn confirm(&mut self) {
        self.confirmed = !self.routines.is_empty();
    }

    pub fn unconfirm(&mut self) {
        self.confirmed = false;
    }

    pub fn count(&self) -> usize {
        self.routines.len()
    }

    pub fn name_at(&self, index: usize) -> &str {
        if index < self.routines.len() {
            &self.routines[index].name
        } else {
            "None"
        }
    }

    pub fn description_at(&self, index: usize) -> &str {
        if index < self.routines.len() {
            &self.routines[index].description
        } else {
            ""
        }
    }

    pub fn get_selection(&self) -> usize {
        self.selection
    }

    pub fn is_confirmed(&self) -> bool {
        self.confirmed
    }

    pub fn get_selected_name(&self) -> &str {
        self.name_at(self.selection)
    }

    pub fn update_from_controller(
        &mut self,
        btn_left_pressed: bool,
        btn_right_pressed: bool,
        btn_a_pressed: bool,
    ) {
        if btn_left_pressed {
            self.select(self.selection as isize - 1);
        }
        if btn_right_pressed {
            self.select(self.selection as isize + 1);
        }
        if btn_a_pressed {
            if self.confirmed {
                self.unconfirm();
            } else {
                self.confirm();
            }
        }
    }
}
