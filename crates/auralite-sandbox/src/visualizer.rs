//! Visualizer for AuraLite worlds.
//! Outputs SVG representations of the simulation state.

use auralite_dynamics::{BodyType, ColliderShape2, World2};
use auralite_math::Vec2;

pub struct SvgVisualizer {
    pub width: f32,
    pub height: f32,
    pub scale: f32,
    pub offset: Vec2,
}

impl SvgVisualizer {
    pub fn new() -> Self {
        Self {
            width: 800.0,
            height: 600.0,
            scale: 50.0,
            offset: Vec2 { x: 400.0, y: 500.0 },
        }
    }

    pub fn render2d(&self, world: &World2) -> String {
        let mut svg = format!(
            r#"<svg width="{}" height="{}" viewBox="0 0 {} {}" xmlns="http://www.w3.org/2000/svg">"#,
            self.width, self.height, self.width, self.height
        );
        svg.push_str(r#"<rect width="100%" height="100%" fill="#);
        svg.push_str("\"#1a1a1a\" />");

        let gy = self.offset.y;
        let ground_tag = format!(
            r#"<line x1="0" y1="{}" x2="{}" y2="{}" stroke="{}" stroke-width="2" />"#,
            gy, self.width, gy, "#333333"
        );
        svg.push_str(&ground_tag);

        for h in world.body_handles() {
            if let Ok(b) = world.body(h) {
                let color = match b.kind {
                    BodyType::Static => "#555555",
                    BodyType::Kinematic => "#44aa99",
                    BodyType::Dynamic => {
                        if b.sleeping {
                            "#444466"
                        } else {
                            "#4499ff"
                        }
                    }
                };

                for c in &b.colliders {
                    let world_pos = b.position + b.rotation.rotate(c.offset);
                    let screen_x = self.offset.x + world_pos.x * self.scale;
                    let screen_y = self.offset.y - world_pos.y * self.scale;

                    match &c.shape {
                        ColliderShape2::Circle(circ) => {
                            let r = circ.radius() * self.scale;
                            let circle_tag = format!(
                                r#"<circle cx="{}" cy="{}" r="{}" fill="{}" opacity="0.8" stroke="white" stroke-width="1" />"#,
                                screen_x, screen_y, r, color
                            );
                            svg.push_str(&circle_tag);
                        }
                        ColliderShape2::Box(bx) => {
                            let hw = bx.half_extents().x * self.scale;
                            let hh = bx.half_extents().y * self.scale;
                            let rect_tag = format!(
                                r#"<rect x="{}" y="{}" width="{}" height="{}" fill="{}" opacity="0.8" stroke="white" stroke-width="1" />"#,
                                screen_x - hw,
                                screen_y - hh,
                                hw * 2.0,
                                hh * 2.0,
                                color
                            );
                            svg.push_str(&rect_tag);
                        }
                        _ => {}
                    }
                }
            }
        }

        svg.push_str("</svg>");
        svg
    }
}
