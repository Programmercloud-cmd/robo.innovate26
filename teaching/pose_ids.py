"""Factory source labels n_j and numeric conveyor/reference labels."""


def normalize_pose_id(identifier):
    text = str(identifier)
    if '_' in text:
        parts = text.split('_')
        if len(parts) == 2 and all(p.isdecimal() for p in parts):
            row, column = map(int, parts)
            if 0 <= row <= 5 and 0 <= column <= 2:
                return f'{row}_{column}'
        raise ValueError('Source pose IDs must be n_j with n=0..5 and j=0..2')
    if not text.isdecimal() or int(text) < 1:
        raise ValueError('Use source n_j or a positive conveyor/reference number')
    number = int(text)
    # Bare numbers 1..6 refer to the original annotated image labels.
    return f'{number-1}_1' if number <= 6 else str(number)


def pose_key(identifier):
    label = normalize_pose_id(identifier)
    return f'point_{label}' if '_' in label else f'point_{int(label):02d}'
