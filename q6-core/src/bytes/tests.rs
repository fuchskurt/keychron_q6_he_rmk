use super::read_array;

#[test]
fn read_array_bounds_and_length() {
    let buf = [10_u8, 20, 30, 40, 50];
    assert_eq!(read_array::<2>(&buf, 1, 3), Some([20, 30]));
    assert_eq!(read_array::<4>(&buf, 0, 4), Some([10, 20, 30, 40]));
    // Out of range.
    assert_eq!(read_array::<2>(&buf, 4, 7), None);
    // Range length does not match N.
    assert_eq!(read_array::<2>(&buf, 0, 3), None);
    // Reversed range yields an empty/invalid slice, not a panic.
    assert_eq!(read_array::<1>(&buf, 3, 1), None);
}
