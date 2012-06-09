(define MAX_SPEED         2.0)
(define MAX_TURN_SPEED    1.6)
(define ROBOT_WIDTH       0.8)
(define SAFETY_MARGIN     0.4)
(define EXTRA_MARGIN      0.1)
(define TURN_INTENSITY    1.7)
(define TURN_RESISTANCE   1.5)
(define MIN_IMPACT_TIME   1.0)

(define EPSILON 0.0001)
(define π 3.14159)
(define else #t)

(define (fold-left func acc l)
  (if (null? l)
	acc
	(fold-left func (func (car l) acc) (cdr l))))

(define (min-list l)
  (if (null? (cdr l))
      (car l)
      (min (car l) (min-list (cdr l)))))

(define (sign n)
  (if (> n 0.0) 1.0 -1.0))

(define (clamp n n0 n1)
  (max (min n n1) n0))

(define (d w α θ)
  (if (> θ α)
     (d w (* -1.0 α) (* -1.0 θ))
     (*
      (/ (sin (- (/ π 2.0) α))
         (sin (- α θ)))
      (/ w 2.0))))

(define (min-dist w α-left α-right ranges)
  (define (corridor-distance θ-range-pair)
    (let ((θ (car θ-range-pair))
          (range (cdr θ-range-pair)))
      (cond
       ((< θ α-right)
        (if (< range (d w α-right θ))
            range
            +inf.0))
       ((and (> θ α-right) (< θ α-left))
        range)
       (else ;(> θ α-left)
        (if (< range (d w α-left θ))
            range
            +inf.0)))))
  (min-list (map corridor-distance ranges)))

(define (forward-speed w α goal-distance goal-flag ranges)
  (let* ((α-left  (max 0.0 α))
         (α-right (min 0.0 α))
         (safe-dist   (- (min-dist w α-left α-right ranges)
                         (* SAFETY_MARGIN 2)))
         (distance    (if goal-flag
                          (min goal-distance safe-dist)
                          safe-dist)))
    (min (/ distance MIN_IMPACT_TIME)
         (* (cos α) MAX_SPEED))))

(define (turn-speed α)
  (* (sign α)
     MAX_TURN_SPEED
     (expt (* (abs (/ α π)) 2.0) (/ 1.0 TURN_INTENSITY))))

(define (turn-speed α)
  (clamp (* (sin α) MAX_TURN_SPEED 1.5) (- MAX_TURN_SPEED) MAX_TURN_SPEED))

(define (goal-progress w goal-α goal-distance α ranges)
  (* (min goal-distance (min-dist (+ EXTRA_MARGIN w) α α ranges))
     (real-part (expt (cos (abs (- goal-α α))) TURN_RESISTANCE))))

(define (find-direction w goal-α goal-distance ranges)
  (define (find-better-α α α-progress-pair)
    (let* ((old-progress (cdr α-progress-pair))
           (progress     (goal-progress w goal-α goal-distance α ranges)))
      (if (> progress old-progress)
          (cons α progress)
          α-progress-pair)))
  (let* ((angles (map car ranges))
         (best-pair (fold-left find-better-α (cons 0.0 -inf.0) angles))
         (best-α    (car best-pair))
         (best-prog (cdr best-pair)))
    best-α))

(define (woah-forward goal-α goal-distance goal-flag ranges)
  (let* ((w (+ ROBOT_WIDTH SAFETY_MARGIN))
         (α (find-direction w goal-α goal-distance ranges))
         (speed (forward-speed w α goal-distance goal-flag ranges))
         (turn  (turn-speed α)))
    (cons speed turn)))

(define (woah-backward goal-α)
  (let ((forward-speed 0.0)
        (turn-speed    (* (sign goal-α) MAX_TURN_SPEED)))
    (cons forward-speed turn-speed)))

(define (woah goal-α goal-distance goal-flag ranges)
  (if (< (abs goal-α) (/ π 2.0))
      (woah-forward goal-α goal-distance goal-flag ranges)
      (woah-backward goal-α)))

(define (configure args)
    (begin
        (set! MAX_SPEED       (list-ref args 0))
        (set! MAX_TURN_SPEED  (list-ref args 1))
        (set! ROBOT_WIDTH     (list-ref args 2))
        (set! SAFETY_MARGIN   (list-ref args 3))
        (set! EXTRA_MARGIN    (list-ref args 4))
        (set! TURN_INTENSITY  (list-ref args 5))
        (set! TURN_RESISTANCE (list-ref args 6))
        (set! MIN_IMPACT_TIME (list-ref args 7))))

(define (get-configuration)
    (list
        MAX_SPEED
        MAX_TURN_SPEED
        ROBOT_WIDTH
        SAFETY_MARGIN
        EXTRA_MARGIN
        TURN_INTENSITY
        TURN_RESISTANCE
        MIN_IMPACT_TIME))
