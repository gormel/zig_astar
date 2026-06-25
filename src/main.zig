const std = @import("std");
const PriorityQueue = std.PriorityQueue;
const ArrayList = std.ArrayList;
const Allocator = std.mem.Allocator;
const Order = std.math.Order;
const testing = std.testing;

pub fn Path(comptime Pos: type) type {
    return struct {
        const Self = Path(Pos);

        path: ArrayList(Pos),
        current: Pos,
        allocator: Allocator,

        pub fn init(current: Pos, allocator: Allocator) Allocator.Error!Self {
            return Self{
                .path = try ArrayList(Pos).initCapacity(allocator, 4),
                .current = current,
                .allocator = allocator,
            };
        }

        pub fn append(self: *Self, gpa: Allocator, pos: Pos) Allocator.Error!void {
            try self.path.append(gpa, pos);
        }

        pub fn deinit(self: *Self) void {
            self.path.deinit(self.allocator);
        }

        pub fn dup(self: *Self) !Self {
            return Self{ .path = try self.path.clone(self.allocator), .current = self.current, .allocator = self.allocator };
        }
    };
}

pub fn Result(comptime Pos: type) type {
    return union(enum) {
        done: Path(Pos),
        neighbors: Pos,
        no_path,
    };
}

pub fn Astar(comptime Pos: type, distance: fn (Pos, Pos) usize) type {
    return struct {
        const Self = @This();
        const NextQueue = PriorityQueue(Path(Pos), Pos, Self.compare);

        next_q: NextQueue,
        seen: ArrayList(Pos),
        start: Pos,
        end: Pos,
        allocator: Allocator,

        pub fn init(start: Pos, allocator: Allocator) Allocator.Error!Self {
            var seen = try ArrayList(Pos).initCapacity(allocator, 4);
            try seen.append(allocator, start);
            return Self{
                .next_q = NextQueue.initContext(start),
                .seen = seen,
                .start = start,
                .end = start,
                .allocator = allocator,
            };
        }

        pub fn deinit(self: *Self) void {
            for (self.next_q.items) |*path| {
                path.deinit();
            }

            self.next_q.deinit(self.allocator);
            self.seen.deinit(self.allocator);
        }

        pub fn pathFind(self: *Self, start: Pos, end: Pos) !Result(Pos) {
            self.next_q.items.len = 0;
            self.seen.items.len = 0;
            try self.seen.append(self.allocator, start);
            self.end = end;
            try self.next_q.push(self.allocator, try Path(Pos).init(start, self.allocator));

            return Result(Pos){ .neighbors = start };
        }

        pub fn step(self: *Self, neighbors: []Pos) !Result(Pos) {
            if (self.next_q.pop()) |popd| {
                var best = popd;
                for (neighbors) |neighbor| {
                    if (std.meta.eql(neighbor, self.end)) {
                        try best.append(self.allocator, best.current);
                        try best.append(self.allocator, self.end);
                        best.current = self.end;
                        return Result(Pos){ .done = best };
                    }

                    var found: bool = false;
                    var i: usize = 0;
                    while (i < self.seen.items.len) : (i += 1) {
                        if (std.meta.eql(neighbor, self.seen.items[i])) {
                            found = true;
                            break;
                        }
                    }
                    if (found) {
                        continue;
                    }
                    try self.seen.append(self.allocator, neighbor);

                    var new_path = try best.dup();
                    try new_path.append(self.allocator, best.current);

                    new_path.current = neighbor;
                    try self.next_q.push(self.allocator, new_path);
                }

                best.deinit();

                const new_best = self.next_q.peek() orelse unreachable;

                return Result(Pos){ .neighbors = new_best.current };
            }

            return Result(Pos).no_path;
        }

        pub fn compare(end: Pos, first: Path(Pos), second: Path(Pos)) Order {
            const firstWeight = first.path.items.len + distance(first.current, end);
            const secondWeight = second.path.items.len + distance(second.current, end);
            return std.math.order(firstWeight, secondWeight);
        }
    };
}

const SimplePos = struct {
    x: isize,
    y: isize,

    pub fn init(x: isize, y: isize) SimplePos {
        return SimplePos{ .x = x, .y = y };
    }
};

fn simple_distance(start: SimplePos, end: SimplePos) usize {
    const y_dist = @abs(start.y - end.y);
    const x_dist = @abs(start.x - end.x);
    return @intCast(@min(x_dist, y_dist));
}

const Map = struct {
    blocked: []const []const bool,

    pub fn init(blocked: []const []const bool) Map {
        return Map{ .blocked = blocked };
    }
};

test "pathfinding" {
    const allocator = std.testing.allocator;

    const PathFinder = Astar(SimplePos, simple_distance);

    const start = SimplePos.init(0, 0);
    const end = SimplePos.init(4, 4);

    var finder = try PathFinder.init(start, allocator);
    defer finder.deinit();

    const blocked: [5][]const bool =
        .{
            &.{ false, true, false, false, false },
            &.{ false, true, false, false, false },
            &.{ false, true, false, false, false },
            &.{ false, true, false, false, false },
            &.{ false, false, false, true, false },
        };
    const map = Map.init(blocked[0..]);

    var result = try finder.pathFind(start, end);
    var neighbors = try ArrayList(SimplePos).initCapacity(allocator, 4);
    defer neighbors.deinit(allocator);

    while (result == .neighbors) {
        const pos = result.neighbors;

        neighbors.clearRetainingCapacity();

        const offsets: [3]isize = .{ -1, 0, 1 };
        for (offsets) |offset_x| {
            for (offsets) |offset_y| {
                const new_x = pos.x + offset_x;
                const new_y = pos.y + offset_y;
                if ((new_x == pos.x and new_y == pos.y) or new_x < 0 or new_y < 0 or new_x > 4 or new_y > 4) {
                    continue;
                }
                if (map.blocked[@intCast(new_y)][@intCast(new_x)]) {
                    continue;
                }
                const next_pos = SimplePos.init(new_x, new_y);
                try neighbors.append(allocator, next_pos);
            }
        }

        result = try finder.step(neighbors.items);
    }

    defer result.done.deinit();

    try testing.expectEqual(Result(SimplePos).done, @as(std.meta.Tag(Result(SimplePos)), result));

    try testing.expectEqual(SimplePos.init(0, 0), result.done.path.items[0]);
    try testing.expectEqual(SimplePos.init(0, 1), result.done.path.items[1]);

    try testing.expectEqual(@as(usize, 3), simple_distance(end, result.done.path.items[1]));
    try testing.expectEqual(SimplePos.init(0, 2), result.done.path.items[2]);

    try testing.expectEqual(@as(usize, 2), simple_distance(end, result.done.path.items[2]));
    try testing.expectEqual(SimplePos.init(0, 3), result.done.path.items[3]);

    try testing.expectEqual(@as(usize, 1), simple_distance(end, result.done.path.items[3]));
    try testing.expectEqual(SimplePos.init(1, 4), result.done.path.items[4]);

    try testing.expectEqual(@as(usize, 0), simple_distance(end, result.done.path.items[4]));
    try testing.expectEqual(SimplePos.init(2, 3), result.done.path.items[5]);

    try testing.expectEqual(SimplePos.init(4, 4), result.done.current);
}
